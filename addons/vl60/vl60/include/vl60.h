//------------------------------------------------------------------------------
//
//      Магистральный электровоз переменного тока ВЛ60.
//      Дополнение для Russian Railway Simulator (RRS)
//
//      (c) RRS development team:
//          Дмитрий Притыкин (maisvendoo),
//          Роман Бирюков (РомычРЖДУЗ)
//
//      Дата: 28/03/2019
//
//------------------------------------------------------------------------------
#ifndef     VL60_H
#define     VL60_H

#include    "vehicle-api.h"

#include    "vl60-signals.h"
#include    "pantograph.h"
#include    "main-switch.h"
#include    "oscillator.h"
#include    "trac-transformer.h"
#include    "phase-splitter.h"
#include    "motor-fan.h"

/*!
 * \class
 * \brief Основной класс, описывающий весь электровоз
 */
//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
class VL60 : public Vehicle
{
public:

    /// Конструктор
    VL60();

    /// Деструктор
    ~VL60();

private:

    enum
    {
        NUM_PANTOGRAPHS = 2,
        WIRE_VOLTAGE = 25000,
        NUM_MOTOR_FANS = 6
    };

    enum
    {
        MV1 = 0,
        MV2 = 1,
        MV3 = 2,
        MV4 = 3,
        MV5 = 4,
        MV6 = 5
    };

    float   Uks;
    float   pant1_pos;
    float   pant2_pos;
    float   gv_pos;
    bool    gv_return;

    float   test_lamp;
    int     sig;

    /// Тригер тумблера "Токоприемники"
    Trigger pants_tumbler;
    /// Триггер тумблера "Токоприемник передний"
    Trigger pant1_tumbler;
    /// Триггер тумблера "Токоприемник задний"
    Trigger pant2_tumbler;

    /// Триггер тумблера "ГВ вкл/выкл"
    Trigger gv_tumbler;

    /// Тригер тумблера "Фазорасщепитель"
    Trigger fr_tumbler;

    /// Триггеры тумблеров управления мотор-вентиляторами
    std::array<Trigger, NUM_MOTOR_FANS> mv_tumblers;

    /// Токоприемники
    std::array<Pantograph *, NUM_PANTOGRAPHS>   pantographs;

    /// Главный выключатель (ГВ)
    MainSwitch      *main_switch;

    /// Механизм киловольтметра КС
    Oscillator      *gauge_KV_ks;

    /// Тяговый трансформатор
    TracTransformer *trac_trans;

    /// Асинхронный расщепитель фаз
    PhaseSplitter   *phase_spliter;

    /// Мотор-вентиляторы
    std::array<MotorFan *, NUM_MOTOR_FANS> motor_fans;

    void initialization();

    void step(double t, double dt);

    void stepPantographsControl(double t, double dt);

    void stepMainSwitchControl(double t, double dt);

    void stepTracTransformer(double t, double dt);

    void stepPhaseSplitter(double t, double dt);

    void stepMotorFans(double t, double dt);

    void stepSignalsOutput();

    bool getHoldingCoilState() const;

    void keyProcess();
};

#endif // VL60_H

