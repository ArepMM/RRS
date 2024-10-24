#ifndef     FREIGHTCAR_H
#define     FREIGHTCAR_H

#include    "vehicle-api.h"

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
class FreightCar : public Vehicle
{
public:

    FreightCar();

    ~FreightCar();

    void initBrakeDevices(double p0, double pBP, double pFL);

private:

    /// Сцепка спереди
    Coupling *coupling_fwd = nullptr;
    QString coupling_module_name = "sa3";
    /// Сцепка сзади
    Coupling *coupling_bwd = nullptr;
    QString coupling_config_name = "sa3";

    /// Расцепной рычаг спереди
    OperatingRod *oper_rod_fwd = nullptr;
    /// Расцепной рычаг сзади
    OperatingRod *oper_rod_bwd = nullptr;

    /// Тормозная магистраль
    Reservoir   *brakepipe = nullptr;
    double      bp_leak = 0.0;

    /// Воздухораспределитель
    AirDistributor  *air_dist = nullptr;
    QString     air_dist_module = "vr483";
    QString     air_dist_config = "vr483";

    /// Электровоздухораспределитель
    ElectroAirDistributor   *electro_air_dist = nullptr;
    QString     electro_air_dist_module = "";
    QString     electro_air_dist_config = "";

    /// Запасный резервуар
    Reservoir   *supply_reservoir = nullptr;
    double      sr_volume = 0.078;
    double      sr_leak = 0.0;

    /// Концевой кран тормозной магистрали спереди
    PneumoAngleCock *anglecock_bp_fwd = nullptr;
    /// Концевой кран тормозной магистрали сзади
    PneumoAngleCock *anglecock_bp_bwd = nullptr;
    QString     anglecock_bp_config = "pneumo-anglecock-BP";

    /// Рукав тормозной магистрали спереди
    PneumoHoseEPB   *hose_bp_fwd = nullptr;
    /// Рукав тормозной магистрали сзади
    PneumoHoseEPB   *hose_bp_bwd = nullptr;
    QString     hose_bp_module = "";
    QString     hose_bp_config = "pneumo-hose-BP";

    /// Авторежим
    BrakeAutoMode *automode = nullptr;
    QString     automode_module = "";
    QString     automode_config = "";

    /// Тормозная рычажная передача
    BrakeMech   *brake_mech = nullptr;
    QString     brake_mech_config = "carbrakes-mech-composite";

    /// Регистратор параметров
    Registrator *registrator = nullptr;
    /// Признак включения регистрации
    bool is_Registrator_on = false;

    void initialization();

    /// Инициализация сцепных устройств
    void initCouplings(const QString &modules_dir, const QString &custom_cfg_dir);

    /// Инициализация тормозного оборудования
    void initBrakesEquipment(const QString &modules_dir, const QString &custom_cfg_dir);

    /// Инициализация ЭПТ
    void initEPB(const QString &modules_dir, const QString &custom_cfg_dir);

    /// Инициализация регистратора параметров в лог-файл
    void initRegistrator(const QString &modules_dir, const QString &custom_cfg_dir);

    /// Предварительные расчёты перед симуляцией
    void preStep(double t);

    /// Предварительный расчёт координат сцепных устройств
    void preStepCouplings(double t);

    /// Шаг моделирования
    void step(double t, double dt);

    /// Моделирование сцепных устройств
    void stepCouplings(double t, double dt);

    /// Моделирование тормозного оборудования
    void stepBrakesEquipment(double t, double dt);

    /// Моделирование ЭПТ
    void stepEPB(double t, double dt);

    /// Вывод параметров в лог-файл
    void stepRegistrator(double t, double dt);

    /// Сигналы для анимации
    virtual void stepSignalsOutput();

    /// Сигналы для озвучки
    virtual void stepSoundsSignals(double t, double dt);

    /// Отладочная строка
    void stepDebugMsg(double t, double dt);

    void keyProcess();

    void loadConfig(QString cfg_path);

};

#endif // FREIGHTCAR_H
