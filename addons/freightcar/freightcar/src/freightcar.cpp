#include    "freightcar.h"
#include    "filesystem.h"

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
FreightCar::FreightCar() : Vehicle ()
{

}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
FreightCar::~FreightCar()
{

}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
void FreightCar::initialization()
{
    FileSystem &fs = FileSystem::getInstance();
    QString modules_dir(fs.getModulesDir().c_str());
    QString custom_cfg_dir(fs.getVehiclesDir().c_str());
    custom_cfg_dir += fs.separator() + config_dir;

    initCouplings(modules_dir, custom_cfg_dir);

    initBrakesEquipment(modules_dir, custom_cfg_dir);

    initEPB(modules_dir, custom_cfg_dir);

    if (is_Registrator_on)
        initRegistrator(modules_dir, custom_cfg_dir);
}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
void FreightCar::preStep(double t)
{
    preStepCouplings(t);
}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
void FreightCar::step(double t, double dt)
{
    stepCouplings(t, dt);

    stepBrakesEquipment(t, dt);

    stepEPB(t, dt);

    stepSignalsOutput();

    stepSoundsSignals(t, dt);

    stepDebugMsg(t, dt);

    if (is_Registrator_on)
        stepRegistrator(t, dt);
}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
void FreightCar::keyProcess()
{

}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
void FreightCar::loadConfig(QString cfg_path)
{
    CfgReader cfg;

    if (cfg.load(cfg_path))
    {
        QString secName = "Vehicle";

        cfg.getDouble(secName, "BrakepipeLeak", bp_leak);

        cfg.getString(secName, "CouplingModule", coupling_module_name);
        cfg.getString(secName, "CouplingConfig", coupling_config_name);

        cfg.getString(secName, "AirDistModule", air_dist_module);
        cfg.getString(secName, "AirDistConfig", air_dist_config);

        cfg.getString(secName, "ElectroAirDistModule", electro_air_dist_module);
        cfg.getString(secName, "ElectroAirDistConfig", electro_air_dist_config);

        cfg.getDouble(secName, "SupplyReservoirVolume", sr_volume);
        cfg.getDouble(secName, "SupplyReservoirLeak", sr_leak);

        cfg.getString(secName, "BrakepipeAnglecockConfig", anglecock_bp_config);

        cfg.getString(secName, "BrakepipeHoseModule", hose_bp_module);
        cfg.getString(secName, "BrakepipeHoseConfig", hose_bp_config);

        cfg.getString(secName, "BrakeAutomodeModule", automode_module);
        cfg.getString(secName, "BrakeAutomodeConfig", automode_config);

        cfg.getString(secName, "BrakeMechConfig", brake_mech_config);

        cfg.getBool(secName, "isRegistratorOn", is_Registrator_on);
    }
}

GET_VEHICLE(FreightCar)
