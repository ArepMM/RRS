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

    float   Uks;
    float   pant1_pos;
    float   pant2_pos;
    float   gv_pos;
    float   gv_return;

    float   test_lamp;
    int     sig;

    Trigger pant1_trig;
    Trigger pant2_trig;

    void step(double t, double dt);

    void keyProcess();
};

#endif // VL60_H

