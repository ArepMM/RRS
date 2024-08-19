#ifndef     TOPOLOGY_H
#define     TOPOLOGY_H

#include    <QObject>

#include    <topology-export.h>
#include    <topology-types.h>
#include    <vehicle-controller.h>
#include    <vehicle.h>

/*!
 * \class
 * \brief Класс, обеспечивающий расчет положения ПЕ на путевой структуре
 */
//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
class TOPOLOGY_EXPORT Topology : public QObject
{
    Q_OBJECT

public:

    Topology(QObject *parent = Q_NULLPTR);

    ~Topology();


    /// Загрузка топологии ж/д полигона
    bool load(QString route_dir);

    /// Общая инициализация
    bool init(const topology_pos_t &tp, std::vector<Vehicle *> *vehicles);

    /// Вернуть контроллер конкретной ПЕ
    VehicleController *getVehicleController(size_t idx);

    QByteArray serialize();

    void deserialize(QByteArray &data);

private:

    /// Контейнер данных по всем траекториям на полигоне
    traj_list_t     traj_list;

    /// Контейнер изостыков
    conn_list_t     joints;

    /// Контейнер стрелок
    conn_list_t     switches;

    /// Контейнер контроллеров ПЕ
    std::vector<VehicleController *> vehicle_control;

    /// Получить список имен всех имеющихся траекторий
    QStringList getTrajNamesList(QString route_dir);

    /// Загрузка топологии
    bool load_topology(QString route_dir);
};

#endif
