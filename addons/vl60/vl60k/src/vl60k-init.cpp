#include    "vl60k.h"

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
void VL60k::initPantographs(const QString &modules_dir, const QString &custom_cfg_dir)
{
    (void) modules_dir;

    for (size_t i = 0; i < NUM_PANTOGRAPHS; ++i)
    {
        pantographs[i] = new Pantograph();
        pantographs[i]->read_config("pantograph", custom_cfg_dir);
    }
}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
void VL60k::initHighVoltageScheme(const QString &modules_dir, const QString &custom_cfg_dir)
{
    (void) modules_dir;

    main_switch = new ProtectiveDevice();
    main_switch->read_config("main-switch", custom_cfg_dir);

    gauge_KV_ks = new Oscillator();
    gauge_KV_ks->read_config("oscillator");

    trac_trans = new TracTransformer();
    trac_trans->read_config("trac-transformer", custom_cfg_dir);
}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
void VL60k::initSupplyMachines(const QString &modules_dir, const QString &custom_cfg_dir)
{
    (void) modules_dir;

    phase_spliter = new PhaseSplitter();
    phase_spliter->read_config("phase-splitter", custom_cfg_dir);

    for (size_t i = 0; i < motor_fans.size(); ++i)
    {
        motor_fans[i] = new ACMotorFan();
        motor_fans[i]->read_config("motor-fan-ac");
    }
}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
void VL60k::initTractionControl(const QString &modules_dir, const QString &custom_cfg_dir)
{
    (void) modules_dir;

    controller = new ControllerKME_60_044();

    main_controller = new EKG_8G();
    main_controller->read_config("ekg-8g", custom_cfg_dir);

    for (size_t i = 0; i < vu.size(); ++i)
    {
        vu[i] = new Rectifier();
        vu[i]->read_config("VU", custom_cfg_dir);
    }

    gauge_KV_motors = new Oscillator();
    gauge_KV_motors->read_config("KV1-osc", custom_cfg_dir);

    for (size_t i = 0; i < motor.size(); ++i)
    {
        motor[i] = new DCMotor();
        motor[i]->setCustomConfigDir(config_dir);
        motor[i]->read_config("HB-412K", custom_cfg_dir);

        overload_relay[i] = new OverloadRelay();
        overload_relay[i]->read_config("PT-140A", custom_cfg_dir);

        linear_contactor[i] = new Relay(NUM_LC_CONTACTS);
        linear_contactor[i]->read_config("bv-8", custom_cfg_dir);
        linear_contactor[i]->setInitContactState(LC_SELF, false);
        linear_contactor[i]->setInitContactState(LC_TED, false);
        linear_contactor[i]->setInitContactState(LC_TED_LAMP, true);
    }
}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
void VL60k::initOtherEquipment(const QString &modules_dir, const QString &custom_cfg_dir)
{
    (void) modules_dir;

    horn = new TrainHorn();
    horn->read_config("train-horn");

    // Система подачи песка
    sand_system = new SandingSystem();
    sand_system->read_config("sanding-system");
    sand_system->setSandMassMax(payload_mass);
    sand_system->setSandLevel(payload_coeff);
/*
    reg = new Registrator();
    reg->setFileName("vl60k-motor");
    reg->init();
*/
}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
void VL60k::initTriggers()
{
    triggers.push_back(&pants_tumbler);
    triggers.push_back(&pant2_tumbler);
    triggers.push_back(&gv_tumbler);
    triggers.push_back(&gv_return_tumbler);
    triggers.push_back(&fr_tumbler);
    triggers.push_back(&mk_tumbler);

    for (size_t i = 0; i < mv_tumblers.size(); ++i)
        triggers.push_back(&mv_tumblers[i]);

    triggers.push_back(&cu_tumbler);

    autoStartTimer = new Timer(0.5);
    connect(autoStartTimer, &Timer::process, this, &VL60k::slotAutoStart);
    start_count = 0;
}
