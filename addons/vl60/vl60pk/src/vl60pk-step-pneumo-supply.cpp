#include    "vl60pk.h"

//------------------------------------------------------------------------
//
//------------------------------------------------------------------------
void VL60pk::stepPneumoSupply(double t, double dt)
{
    press_reg->setFLpressure(main_reservoir->getPressure());
    press_reg->step(t, dt);

    double U_power = phase_spliter->getU_out()
                    * static_cast<double>(mk_tumbler.getState())
                    * press_reg->getState();
    motor_compressor->setFLpressure(main_reservoir->getPressure());
    motor_compressor->setPowerVoltage(U_power);
    motor_compressor->step(t, dt);

    double FL_flow = 0.0;
    FL_flow += motor_compressor->getFLflow();
    FL_flow += brake_lock->getFLflow();
    FL_flow += bc_pressure_relay->getFLflow();
    FL_flow += anglecock_fl_fwd->getFlowToPipe();
    FL_flow += anglecock_fl_bwd->getFlowToPipe();
    main_reservoir->setFlow(FL_flow);
    main_reservoir->step(t, dt);

    // Концевые краны питательной магистрали
    anglecock_fl_fwd->setPipePressure(main_reservoir->getPressure());
    anglecock_fl_fwd->setHoseFlow(hose_fl_fwd->getFlow());
    anglecock_fl_fwd->step(t, dt);
    anglecock_fl_bwd->setPipePressure(main_reservoir->getPressure());
    anglecock_fl_bwd->setHoseFlow(hose_fl_bwd->getFlow());
    anglecock_fl_bwd->step(t, dt);

    // Рукава тормозной питательной магистрали
    hose_fl_fwd->setPressure(anglecock_fl_fwd->getPressureToHose());
    hose_fl_fwd->setFlowCoeff(anglecock_fl_fwd->getFlowCoeff());
    hose_fl_fwd->step(t, dt);
    hose_fl_bwd->setPressure(anglecock_fl_bwd->getPressureToHose());
    hose_fl_bwd->setFlowCoeff(anglecock_fl_bwd->getFlowCoeff());
    hose_fl_bwd->step(t, dt);
}