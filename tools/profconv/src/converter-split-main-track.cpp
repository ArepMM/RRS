#include    "converter.h"

#include    <QFile>

#include    "path-utils.h"

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
void ZDSimConverter::findSplitsMainTrajectories(const int &dir)
{
    if (dir > 0)
    {
        bool was_1_track = false;
        size_t id = 0;
        for (auto it = tracks_data1.begin(); it != tracks_data1.end(); ++it)
        {
            zds_track_t track = *it;
            split_zds_trajectory_t split = split_zds_trajectory_t();

            // Проверяем однопутный участок
            for (auto it2 = tracks_data2.begin(); it2 != tracks_data2.end(); ++it2)
            {
                zds_track_t track2 = *it2;
                if (length(track.begin_point - track2.begin_point) < 0.1)
                {
                    if ((!was_1_track) && (id != 0))
                    {
                        // Начало однопутного участка
                        split.split_type.push_back(split_zds_trajectory_t::SPLIT_2_1);
                        // Добавляем разделение и в соседний главный путь
                        split_zds_trajectory_t split2 = split_zds_trajectory_t();
                        split2.track_id = track2.prev_uid;
                        split2.point = track2.begin_point;
                        split2.railway_coord = track2.railway_coord;
                        split2.split_type.push_back(split_zds_trajectory_t::SPLIT_2_1);
                        addOrCreateSplit(split_data2, split2);
                    }

                    if (length(track.end_point - track2.end_point) < 0.1)
                    {
                        // Однопутный участок
                        it2->id_at_track1 = id;
                        was_1_track = true;
                    }
                    else
                    {
                        // Конец однопутного участка
                        split.split_type.push_back(split_zds_trajectory_t::SPLIT_1_2);
                        // Добавляем разделение и в соседний главный путь
                        split_zds_trajectory_t split2 = split_zds_trajectory_t();
                        split2.track_id = track2.prev_uid;
                        split2.point = track2.begin_point;
                        split2.railway_coord = track2.railway_coord;
                        split2.split_type.push_back(split_zds_trajectory_t::SPLIT_1_2);
                        addOrCreateSplit(split_data2, split2);
                        was_1_track = false;
                    }
                    break;
                }
            }

            // Съезды между главными путями
            // Добавляем съезды "2-2", найденные в branch_tracks
            bool is_branch = false;
            for (auto branch22 : branch_2minus2_data)
            {
                if (branch22->id1 == id)
                {
                    is_branch = true;

                    // Разделение в начале данного трека
                    split.split_type.push_back(split_zds_trajectory_t::SPLIT_2PLUS2);

                    // Добавляем разделение и в соседний главный путь
                    split_zds_trajectory_t split2 = split_zds_trajectory_t();
                    split2.track_id = branch22->id2;
                    split2.point = tracks_data2[branch22->id2].begin_point;
                    split2.railway_coord = tracks_data2[branch22->id2].railway_coord;
                    split2.split_type.push_back(split_zds_trajectory_t::SPLIT_2PLUS2);
                    addOrCreateSplit(split_data2, split2);

                    break;
                }
            }
            // Проверяем съезды "2-2", записанные в route.trk
            if ((track.arrows == "2-2") && (!is_branch))
            {
                // Разделение в начале данного трека
                split.split_type.push_back(split_zds_trajectory_t::SPLIT_2PLUS2);

                // Добавляем разделение и в соседний главный путь
                float coord;
                zds_track_t track2 = getNearestTrack(track.end_point, tracks_data2, coord);
                bool near_end = (coord > (track2.trajectory_coord + 0.5 * track2.length));
                size_t id2 = near_end ? track2.prev_uid + 1 : track2.prev_uid;

                split_zds_trajectory_t split2 = split_zds_trajectory_t();
                split2.track_id = id2;
                split2.point = tracks_data2[id2].begin_point;
                split2.railway_coord = tracks_data2[id2].railway_coord;
                split2.split_type.push_back(split_zds_trajectory_t::SPLIT_2MINUS2);
                addOrCreateSplit(split_data2, split2);

                // Добавляем съезд в массив траекторий съездов
                zds_branch_2_2_t branch2minus2 = zds_branch_2_2_t();
                branch2minus2.id1 = id;
                branch2minus2.id2 = id2;
                branch_2minus2_data.push_back(new zds_branch_2_2_t(branch2minus2));
            }
            // Добавляем съезды "2+2", найденные в branch_tracks
            is_branch = false;
            for (auto branch22 : branch_2plus2_data)
            {
                if (branch22->id1 == id + 1)
                {
                    is_branch = true;

                    // Разделение в конце данного трека - в начале следующего
                    split_zds_trajectory_t split_at_next = split_zds_trajectory_t();
                    split_at_next.track_id = id + 1;
                    split_at_next.point = track.end_point;
                    split_at_next.railway_coord = track.railway_coord_end;
                    split_at_next.split_type.push_back(split_zds_trajectory_t::SPLIT_2PLUS2);
                    // Проверяем светофор на возвращении с branch_track
                    if ((branch22->is_bwd) && (!branch22->branch_point_bwd.signal_liter.empty()))
                    {
                        split_at_next.split_type.push_back(split_zds_trajectory_t::SPLIT_SIGNAL_BWD);
                        split_at_next.signal_bwd_liter = branch22->branch_point_bwd.signal_liter;
                    }
                    addOrCreateSplit(split_data1, split_at_next);

                    // Добавляем разделение и в соседний главный путь
                    split_zds_trajectory_t split2 = split_zds_trajectory_t();
                    split2.track_id = branch22->id2;
                    split2.point = tracks_data2[branch22->id2].begin_point;
                    split2.railway_coord = tracks_data2[branch22->id2].railway_coord;
                    split2.split_type.push_back(split_zds_trajectory_t::SPLIT_2PLUS2);
                    // Проверяем светофор на возвращении с branch_track
                    if ((branch22->is_fwd) && (!branch22->branch_point_fwd.signal_liter.empty()))
                    {
                        split2.split_type.push_back(split_zds_trajectory_t::SPLIT_SIGNAL_FWD);
                        split2.signal_fwd_liter = branch22->branch_point_fwd.signal_liter;
                    }
                    addOrCreateSplit(split_data2, split2);

                    break;
                }
            }
            // Проверяем съезды "2+2", записанные в route.trk
            if ((track.arrows == "2+2") && (!is_branch))
            {
                // Разделение в конце данного трека - в начале следующего
                split_zds_trajectory_t split_at_next = split_zds_trajectory_t();
                split_at_next.track_id = id + 1;
                split_at_next.point = track.end_point;
                split_at_next.railway_coord = track.railway_coord_end;
                split_at_next.split_type.push_back(split_zds_trajectory_t::SPLIT_2PLUS2);
                addOrCreateSplit(split_data1, split_at_next);

                // Добавляем разделение и в соседний главный путь
                float coord;
                zds_track_t track2 = getNearestTrack(track.begin_point, tracks_data2, coord);
                bool near_end = (coord > (track2.trajectory_coord + 0.5 * track2.length));
                size_t id2 = near_end ? track2.prev_uid + 1 : track2.prev_uid;

                split_zds_trajectory_t split2 = split_zds_trajectory_t();
                split2.track_id = id2;
                split2.point = tracks_data2[id2].begin_point;
                split2.railway_coord = tracks_data2[id2].railway_coord;
                split2.split_type.push_back(split_zds_trajectory_t::SPLIT_2PLUS2);
                addOrCreateSplit(split_data2, split2);

                // Добавляем съезд в массив траекторий съездов
                zds_branch_2_2_t branch2plus2 = zds_branch_2_2_t();
                branch2plus2.id1 = id + 1;
                branch2plus2.id2 = id2;
                branch_2plus2_data.push_back(new zds_branch_2_2_t(branch2plus2));
            }

            // Стрелки на боковые пути
            // Добавляем съезды на боковые пути, найденные в branch_tracks
            is_branch = false;
            for (auto branch : branch_track_data1)
            {
                if (branch->id_begin == id)
                {
                    is_branch = true;
                    split.split_type.push_back(split_zds_trajectory_t::SPLIT_TO_SIDE);
                }
            }
            // Добавляем стрелки "+2" или "-2", записанные в route.trk
            if ( ((track.arrows == "+2") || (track.arrows == "-2")) && (!is_branch) )
            {
                split.split_type.push_back(split_zds_trajectory_t::SPLIT_TO_SIDE);
            }
            // Добавляем съезды с боковых путей, найденных в branch_tracks
            is_branch = false;
            for (auto branch : branch_track_data1)
            {
                if (branch->id_end == id + 1)
                {
                    is_branch = true;

                    // Разделение в конце данного трека - в начале следующего
                    split_zds_trajectory_t split_at_next = split_zds_trajectory_t();
                    split_at_next.track_id = id + 1;
                    split_at_next.point = track.end_point;
                    split_at_next.railway_coord = track.railway_coord_end;
                    split_at_next.split_type.push_back(split_zds_trajectory_t::SPLIT_FROM_SIDE);
                    addOrCreateSplit(split_data1, split_at_next);
                }
            }
            // Добавляем стрелки "2+" или "2-", записанные в route.trk
            if ( ((track.arrows == "2+") || (track.arrows == "2-")) && (!is_branch) )
            {
                // Разделение в конце данного трека - в начале следующего
                split_zds_trajectory_t split_at_next = split_zds_trajectory_t();
                split_at_next.track_id = id + 1;
                split_at_next.point = track.end_point;
                split_at_next.railway_coord = track.railway_coord_end;
                split_at_next.split_type.push_back(split_zds_trajectory_t::SPLIT_FROM_SIDE);
                addOrCreateSplit(split_data1, split_at_next);
            }

            for (auto s_it = signals_data1.begin(); s_it != signals_data1.end(); ++s_it)
            {
                if ((*s_it).track_id == id)
                {
                    split.split_type.push_back(split_zds_trajectory_t::SPLIT_SIGNAL_FWD);
                    split.signal_fwd_type = (*s_it).type;
                    split.signal_fwd_liter = (*s_it).liter;
                    break;
                }
            }

            if (!split.split_type.empty())
            {
                split.track_id = id;
                split.point = track.begin_point;
                split.railway_coord = track.railway_coord;
                addOrCreateSplit(split_data1, split);
            }

            ++id;
        }
    }
    else
    {
        size_t id = 0;
        for (auto it = tracks_data2.begin(); it != tracks_data2.end(); ++it)
        {
            zds_track_t track = *it;
            split_zds_trajectory_t split = split_zds_trajectory_t();

            // Стрелки на боковые пути
            // Добавляем съезды на боковые пути, найденные в branch_tracks
            bool is_branch = false;
            for (auto branch : branch_track_data2)
            {
                if (branch->id_end == id)
                {
                    is_branch = true;
                    split.split_type.push_back(split_zds_trajectory_t::SPLIT_TO_SIDE);
                }
            }
            // Добавляем стрелки "+2" или "-2", записанные в route.trk
            if ( ((track.arrows == "+2") || (track.arrows == "-2")) && (!is_branch) )
            {
                split.split_type.push_back(split_zds_trajectory_t::SPLIT_TO_SIDE);
            }
            // Добавляем съезды с боковых путей, найденных в branch_tracks
            is_branch = false;
            for (auto branch : branch_track_data2)
            {
                if (branch->id_begin == id + 1)
                {
                    is_branch = true;

                    // Разделение в конце данного трека - в начале следующего
                    split_zds_trajectory_t split_at_next = split_zds_trajectory_t();
                    split_at_next.track_id = id + 1;
                    split_at_next.point = track.end_point;
                    split_at_next.railway_coord = track.railway_coord_end;
                    split_at_next.split_type.push_back(split_zds_trajectory_t::SPLIT_FROM_SIDE);
                    if (track.id_at_track1 == -1)
                    {
                        addOrCreateSplit(split_data2, split_at_next);
                    }
                    else
                    {
                        split_at_next.track_id = track.id_at_track1;
                        addOrCreateSplit(split_data1, split_at_next);
                    }
                }
            }
            // Добавляем стрелки "2+" или "2-", записанные в route.trk
            if ( ((track.arrows == "2+") || (track.arrows == "2-")) && (!is_branch))
            {
                // Разделение в конце данного трека - в начале следующего
                split_zds_trajectory_t split_at_next = split_zds_trajectory_t();
                split_at_next.track_id = id + 1;
                split_at_next.point = track.end_point;
                split_at_next.railway_coord = track.railway_coord_end;
                split_at_next.split_type.push_back(split_zds_trajectory_t::SPLIT_FROM_SIDE);
                if (track.id_at_track1 == -1)
                {
                    addOrCreateSplit(split_data2, split_at_next);
                }
                else
                {
                    split_at_next.track_id = track.id_at_track1;
                    addOrCreateSplit(split_data1, split_at_next);
                }
            }

            for (auto s_it = signals_data2.begin(); s_it != signals_data2.end(); ++s_it)
            {
                if ((*s_it).track_id + 1 == id)
                {
                    split.split_type.push_back(split_zds_trajectory_t::SPLIT_SIGNAL_BWD);
                    split.signal_bwd_type = (*s_it).type;
                    split.signal_bwd_liter = (*s_it).liter;
                    break;
                }
            }

            if (!split.split_type.empty())
            {
                split.track_id = id;
                split.point = track.begin_point;
                split.railway_coord = track.railway_coord;
                if (track.id_at_track1 == -1)
                {
                    addOrCreateSplit(split_data2, split);
                }
                else
                {
                    split.track_id = track.id_at_track1;
                    addOrCreateSplit(split_data1, split);
                }
            }

            ++id;
        }
        std::sort(split_data1.begin(), split_data1.end(), split_zds_trajectory_t::compare_by_track_id);
        std::sort(split_data2.begin(), split_data2.end(), split_zds_trajectory_t::compare_by_track_id);
    }
}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
void ZDSimConverter::writeSplits(const int &dir)
{
    // Вывод в файл, для отладки
    std::string path;
    route_connectors_t* data;
    if (dir > 0)
    {
        path = compinePath(toNativeSeparators(topologyDir), "split1.conf");
        data = &split_data1;
    }
    else
    {
        path = compinePath(toNativeSeparators(topologyDir), "split2.conf");
        data = &split_data2;
    }

    QFile file_old(QString(path.c_str()));
    if (file_old.exists())
        file_old.rename( QString((path + ".bak").c_str()) );

    QFile file(QString(path.c_str()));
    if (!file.open(QIODevice::WriteOnly | QIODevice::Text))
        return;

    QTextStream stream(&file);
    stream.setEncoding(QStringConverter::Utf8);
    stream.setRealNumberNotation(QTextStream::FixedNotation);

    for (auto it = data->begin(); it != data->end(); ++it)
    {
        stream << (*it)->point.x << ";"
               << (*it)->point.y << ";"
               << (*it)->point.z << ";"
               << (*it)->track_id << ";"
               << static_cast<int>(round((*it)->railway_coord)) << ";";
        for (auto type_it : (*it)->split_type)
        {
            switch (type_it)
            {
            case split_zds_trajectory_t::SPLIT_2MINUS2:
            {
                stream << "splitby2minus2;";
                break;
            }
            case split_zds_trajectory_t::SPLIT_2PLUS2:
            {
                stream << "splitby2plus2;";
                break;
            }
            case split_zds_trajectory_t::SPLIT_2_1:
            {
                stream << "splitby1begin;";
                break;
            }
            case split_zds_trajectory_t::SPLIT_1_2:
            {
                stream << "splitby2begin;";
                break;
            }
            case split_zds_trajectory_t::SPLIT_TO_SIDE:
            case split_zds_trajectory_t::SPLIT_FROM_SIDE:
            {
                stream << "splitbyside_" << type_it << ";";
                break;
            }
            case split_zds_trajectory_t::SPLIT_SIGNAL_FWD:
            {
                stream << "splitsignF_" << (*it)->signal_fwd_liter.c_str() << ";";
                break;
            }
            case split_zds_trajectory_t::SPLIT_SIGNAL_BWD:
            {
                stream << "splitsignB_" << (*it)->signal_bwd_liter.c_str() << ";";
                break;
            }
            default: stream << "WARN_" << type_it << ";";
            }
        }
        stream << "\n";
    }

    file.close();
}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
void ZDSimConverter::addOrCreateSplit(route_connectors_t &split_data, const split_zds_trajectory_t &split_point)
{
    if (split_point.split_type.empty())
        return;

    auto exist_it = split_data.begin();
    while (exist_it != split_data.end())
    {
        if (((*exist_it)->track_id == split_point.track_id))
        {
            for (auto type_it : split_point.split_type)
            {
                (*exist_it)->split_type.push_back(type_it);
                if (type_it == split_zds_trajectory_t::SPLIT_SIGNAL_FWD)
                {
                    (*exist_it)->signal_bwd_type = split_point.signal_fwd_type;
                    (*exist_it)->signal_bwd_liter = split_point.signal_fwd_liter;
                }
                if (type_it == split_zds_trajectory_t::SPLIT_SIGNAL_BWD)
                {
                    (*exist_it)->signal_bwd_type = split_point.signal_bwd_type;
                    (*exist_it)->signal_bwd_liter = split_point.signal_bwd_liter;
                }
            }
            return;
        }

        ++exist_it;
    }

    split_data.push_back(new split_zds_trajectory_t(split_point));
}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
void ZDSimConverter::splitMainTrajectory(const int &dir)
{

}

