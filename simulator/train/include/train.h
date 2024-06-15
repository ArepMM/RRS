//------------------------------------------------------------------------------
//
//      Common train's model dynamics
//      (c) maisvendoo, 04/09/2018
//      Developer: Dmitry Pritykin
//
//------------------------------------------------------------------------------
/*!
 * \file
 * \brief Common train's model dynamics
 * \copyright maisvendoo
 * \author Dmitry Pritykin
 * \date 04/09/2018
 */

#ifndef     TRAIN_H
#define     TRAIN_H

#include    "filesystem.h"
#include    "init_data.h"
#include    "simulator-update-struct.h"
#include    "ode-system.h"
#include    "vehicle.h"
#include    "device-list.h"
#include    "device-joint.h"
#include    "solver.h"
#include    "solver-config.h"
#include    "profile.h"
#include    "sound-manager.h"

#include    <QByteArray>

#if defined(TRAIN_LIB)
    #define TRAIN_EXPORT    Q_DECL_EXPORT
#else
    #define TRAIN_EXPORT    Q_DECL_IMPORT
#endif

/*!
 * \class
 * \brief Common train model
 */
//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
class TRAIN_EXPORT Train : public OdeSystem
{
    Q_OBJECT

public:

    /// Constructor
    explicit Train(Profile *profile, QObject *parent = Q_NULLPTR);
    /// Destructor
    virtual ~Train();

    /// Train initialization
    bool init(const init_data_t &init_data);

    /// Calculation of right part motion ODE's
    void calcDerivative(state_vector_t &Y, state_vector_t &dYdt, double t, double dt);

    /// Action before time step
    void preStep(double t, simulator_update_t &update_data);

    /// Integration step
    bool step(double t, double &dt);

    /// Integration step for vehicles ODE's
    void vehiclesStep(double t, double dt);

    void inputProcess();

    /// Action after integration step
    void postStep(double t);

    /// Get first vehicle
    Vehicle *getFirstVehicle() const;

    /// Get last vehicle
    Vehicle *getLastVehicle() const;

    double getVelocity(size_t i) const;

    /// Get train mass
    double getMass() const;
    /// Get train length
    double getLength() const;

    size_t getVehiclesNumber() const;

    QString getClientName();

    QString getTrainID();

    int getDirection() const;

    std::vector<Vehicle *> *getVehicles();

signals:

    void sendDataToVehicle(QByteArray data);

private:

    /// Train mass
    double          trainMass;
    /// Train length
    double          trainLength;

    /// Order of system ODE motion
    size_t          ode_order;

    /// Direction of motion on railway
    int             dir;

    /// Vehicle which selected by user for view
    int             current_vehicle;

    /// Vehicle which selected by user for control
    int             controlled_vehicle;

    /// Profile manager
    Profile     *profile;

    /// Coefficient to friction between wheel and rail
    double      coeff_to_wheel_rail_friction;

    /// Charging pressure
    double      charging_pressure;

    /// No air flag (for empty air system on start)
    bool        no_air;

    /// Initial main reservoir pressure
    double      init_main_res_pressure;

    /// Motion ODE's solver
    Solver      *train_motion_solver;

    /// Sound manager
    SoundManager *soundMan;

    /// Имя сетевого клиента для ВЖД
    QString     client_name;

    /// Идентификатор поезда для ВЖД
    QString     train_id;

    /// All train's vehicles
    std::vector<Vehicle *> vehicles;

    /// All joints between neighbor vehicles
    std::vector<std::vector<Joint *>> joints_list;

    /// Solver's configuration
    solver_config_t solver_config;

    /// Train's loading
    bool loadTrain(QString cfg_path, const init_data_t &init_data);
    /// Joints loading
    bool loadJoints();
    /// Joint module loading
    void loadJointModule(Device *con_fwd, Device *con_bwd, std::vector<Joint *> &joints);

    /// Set initial conditions
    void setInitConditions(const init_data_t &init_data);

    /// Initialization of vehicles brakes
    void initVehiclesBrakes();
};

#endif // TRAIN_H
