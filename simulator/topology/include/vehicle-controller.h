#ifndef     VEHICLE_CONTROLLER_H
#define     VEHICLE_CONTROLLER_H

#include    <QObject>

#include    <topology-types.h>
#include    <topology-export.h>
#include    <profile-point.h>
#include    <device-list.h>
#include    <device-joint.h>

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
class TOPOLOGY_EXPORT VehicleController : public QObject
{
    Q_OBJECT

public:

    VehicleController(QObject *parent = Q_NULLPTR);

    virtual ~VehicleController();

    /// Устанавливаем текущую дуговую координату ПЕ
    void setIndex(size_t idx);

    /// Устанавливаем текущую дуговую координату ПЕ
    void setRailwayCoord(double x);

    /// Задание начальной дуговой координаты ПЕ
    void setInitRailwayCoord(double x);

    /// Задание начальной траектории на которой находится ПЕ
    void setInitCurrentTraj(Trajectory *traj);

    /// Задаем координату ПЕ вдоль траектории
    void setTrajCoord(double traj_coord) { this->traj_coord = traj_coord; }

    /// Устанавливаем направление движения по траектории
    void setDirection(int dir) { this->dir = dir; }

    /// Задать оборудование ПЕ, взаимодействующее с путевой инфраструктурой
    void setVehicleRailwayConnectors(device_list_t *devices);

    /// Указатели на оборудование ПЕ, взаимодействующее с путевой инфраструктурой
    device_list_t *getVehicleRailwayConnectors();

    /// Вернуть структуру, определяющую положение ПЕ в пространстве
    profile_point_t getPosition(int dir);

    /// Вернуть указатель на текущую траекторию
    Trajectory *getCurrentTraj() const { return current_traj; }

    /// Вернтуть текущую координату вдоль траектории
    double getTrajCoord() const { return traj_coord; }

protected:

    /// Индекс данной ПЕ в симуляции
    size_t index;

    /// Текущее значение дуговой координаты ПЕ
    double  x_cur = 0.0;

    /// Направление движения
    int    dir = 1;

    /// Координата, в пределах текущей траектории
    double traj_coord = 0.0;

    /// Оборудование ПЕ, взаимодействующее с путевой инфраструктурой
    device_list_t *devices;

    /// Текущая траектория ПЕ
    Trajectory *current_traj = Q_NULLPTR;

    /// Предыдущая траектория ПЕ (за коннектором сзади по ходу движения)
    Trajectory *prev_traj = Q_NULLPTR;
};

#endif // VEHICLE_CONTROLLER_H
