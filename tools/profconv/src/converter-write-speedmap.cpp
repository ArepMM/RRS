#include    "converter.h"

#include    <QDir>
#include    <QVariant>

#include    "path-utils.h"
#include    "CfgEditor.h"

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
void ZDSimConverter::writeSpeedmap()
{
    if (speedmap_data.empty())
        return;

    QString node = CONFIGNODE_TRAJ_3LVL.c_str();
    QString node2 = CONFIGNODE_TRAJ_2LVL.c_str();
    QString node_speed = "SpeedLimit";
    QString node2_speed = "SpeedMap";
    for (auto speedmap_element : speedmap_data)
    {
        if (speedmap_element->trajectories_names.empty() || speedmap_element->speedmap_elements.empty())
            continue;

        int coord_begin = speedmap_element->speedmap_elements.front().railway_coord_begin;
        int coord_end = speedmap_element->speedmap_elements.back().railway_coord_end;
        QString filename = speedmap_element->name_prefix.c_str();
        if (coord_begin >= 0)
            filename += QString("_%1_%2").arg(coord_begin).arg(coord_end);
        filename += ".xml";
        std::string path = compinePath(speedmapDir, filename.toStdString());

        CfgEditor editor;
        editor.openFileForWrite(QString(path.c_str()));
        editor.setIndentationFormat(-1);

        // Список траекторий
        for (auto tn : speedmap_element->trajectories_names)
        {
            FieldsDataList fdl = { QPair<QString, QString>(node, QString(tn.c_str())) };

            editor.writeFile(node2, fdl);
        }

        for (auto se : speedmap_element->speedmap_elements)
        {
            QString field_value = QString("%1 %2 %3")
                                      .arg(se.limit)
                                      .arg(se.railway_coord_begin)
                                      .arg(se.railway_coord_end);
            FieldsDataList fdl = { QPair<QString, QString>(node_speed, field_value) };

            editor.writeFile(node2_speed, fdl);
        }
        editor.closeFileAfterWrite();
    }
}
