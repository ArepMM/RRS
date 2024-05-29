#include    "vl60pk.h"

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
void VL60pk::keyProcess()
{
    if (autoStartTimer->isStarted())
        return;

    // Управление тумблером "Токоприемники"
    if (getKeyState(KEY_U))
    {
        if (isShift())
            pants_tumbler.set();
        else
            pants_tumbler.reset();
    }

    // Подъем/опускание переднего токоприемника
    if (getKeyState(KEY_I))
    {
        // Переводим тумблер в нужное фиксированное положение
        if (isShift())
            pant1_tumbler.set();
        else
            pant1_tumbler.reset();
    }

    // Подъем/опускание заднего токоприемника
    if (getKeyState(KEY_O))
    {
        // Переводим тумблер в нужное фиксированное положение
        if (isShift())
            pant2_tumbler.set();
        else
            pant2_tumbler.reset();

    }

    // Включение/выключение ГВ
    if (getKeyState(KEY_P))
    {
        if (isShift())
            gv_tumbler.set();
        else
            gv_tumbler.reset();
    }

    // Возврат защиты
    if (getKeyState(KEY_K))
        gv_return_tumbler.set();
    else
        gv_return_tumbler.reset();

    // Включение/выключение расщепителя фаз
    if (getKeyState(KEY_T))
    {
        if (isShift())
            fr_tumbler.set();
        else
            fr_tumbler.reset();
    }

    // Включение/выключение мотор-верниляторов

    // МВ1
    if (getKeyState(KEY_R))
    {
        if (isShift())
            mv_tumblers[MV1].set();
        else
            mv_tumblers[MV1].reset();
    }

    // МВ2
    if (getKeyState(KEY_F))
    {
        if (isShift())
            mv_tumblers[MV2].set();
        else
            mv_tumblers[MV2].reset();
    }

    // МВ3
    if (getKeyState(KEY_Y))
    {
        if (isShift())
            mv_tumblers[MV3].set();
        else
            mv_tumblers[MV3].reset();
    }

    // МВ4
    if (getKeyState(KEY_5))
    {
        if (isShift())
            mv_tumblers[MV4].set();
        else
            mv_tumblers[MV4].reset();
    }

    // МВ5
    if (getKeyState(KEY_6))
    {
        if (isShift())
            mv_tumblers[MV5].set();
        else
            mv_tumblers[MV5].reset();
    }

    // МВ6
    if (getKeyState(KEY_7))
    {
        if (isShift())
            mv_tumblers[MV6].set();
        else
            mv_tumblers[MV6].reset();
    }

    // Включение/выключение мотор-компрессора
    if (getKeyState(KEY_E))
    {
        if (isShift())
            mk_tumbler.set();
        else
            mk_tumbler.reset();
    }

    // Включение/выключение цепей управления
    if (getKeyState(KEY_J))
    {
        if (isShift())
            cu_tumbler.set();
        else
            cu_tumbler.reset();
    }

    // Нажатие РБ-1
    if (getKeyState(KEY_Z))
        rb[RB_1].set();
    else
        rb[RB_1].reset();

    // Нажатие РБС

    // Если активна РБС на внешнем пульте
    if (control_signals.analogSignal[CS_RBS].is_active)
    {
        // реагируем на состояние РБС на внешнем пульте
        if (static_cast<bool>(control_signals.analogSignal[CS_RBS].cur_value))
            rb[RBS].set();
        else
            rb[RBS].reset();
    }
    else // иначе
    {
        // обрабатываем клавиши
        if (getKeyState(KEY_M))
            rb[RBS].set();
        else
            rb[RBS].reset();
    }

    // Нажатие РБП
    if (getKeyState(KEY_Q))
        rb[RBP].set();
    else
        rb[RBP].reset();

    if (getKeyState(KEY_R))
    {
        if (isAlt() && !autoStartTimer->isStarted())
            autoStartTimer->start();
    }

    // Включение/выключение ЭПТ
    if (getKeyState(KEY_V))
    {
        if (isShift())
            epb_switch.set();
        else
            epb_switch.reset();
    }

    // Сцепка/Отцепка спереди
    if (getKeyState(KEY_X) && (!isShift()))
    {
        if (isControl())
        {
            anglecock_bp_fwd->close();
            anglecock_fl_fwd->close();
            anglecock_bc_fwd->close();
            hose_bp_fwd->disconnect();
            hose_fl_fwd->disconnect();
            hose_bc_fwd->disconnect();
        }
        else
        {
            if (coupling_fwd->isCoupled())
            {
                hose_bp_fwd->connect();
                hose_fl_fwd->connect();
                hose_bc_fwd->connect();

                if (hose_bp_fwd->isLinked())
                    anglecock_bp_fwd->open();
                else
                    anglecock_bp_fwd->close();

                if (hose_fl_fwd->isLinked())
                    anglecock_fl_fwd->open();
                else
                    anglecock_fl_fwd->close();

                if (hose_bc_fwd->isLinked())
                    anglecock_bc_fwd->open();
                else
                    anglecock_bc_fwd->close();
            }
        }
    }

    // Сцепка/Отцепка сзади
    if (getKeyState(KEY_C)&& (!isShift()))
    {
        if (isControl())
        {
            anglecock_bp_bwd->close();
            anglecock_fl_bwd->close();
            anglecock_bc_bwd->close();
            hose_bp_bwd->disconnect();
            hose_fl_bwd->disconnect();
            hose_bc_bwd->disconnect();
        }
        else
        {
            if (coupling_bwd->isCoupled())
            {
                hose_bp_bwd->connect();
                hose_fl_bwd->connect();
                hose_bc_bwd->connect();

                if (hose_bp_bwd->isLinked())
                    anglecock_bp_bwd->open();
                else
                    anglecock_bp_bwd->close();

                if (hose_fl_bwd->isLinked())
                    anglecock_fl_bwd->open();
                else
                    anglecock_fl_bwd->close();

                if (hose_bc_bwd->isLinked())
                    anglecock_bc_bwd->open();
                else
                    anglecock_bc_bwd->close();
            }
        }
    }
}
