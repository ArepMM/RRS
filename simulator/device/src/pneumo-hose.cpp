#include    "pneumo-hose.h"

#include    "CfgReader.h"

#include    "physics.h"

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
PneumoHose::PneumoHose(QObject *parent) : Device(parent)
{
    name = QString("BP");

    input_signals.resize(HOSE_SIZE_OF_INPUTS);
    output_signals.resize(HOSE_SIZE_OF_OUTPUTS);

    std::fill(input_signals.begin(), input_signals.end(), 0.0);
    std::fill(output_signals.begin(), output_signals.end(), 0.0);

    ref_state.setSpringFirst();
    ref_state.setSpringLast();
    ref_state.setInitPosition(1);
}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
PneumoHose::~PneumoHose()
{

}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
void PneumoHose::setKeySymbolConnect(std::uint16_t key_symbol)
{
    ref_state.setKeySymbolIncrease(key_symbol);
}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
void PneumoHose::setKeyModifierConnect(std::uint16_t key_modifier)
{
    ref_state.setKeyModifierIncrease(key_modifier);
}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
void PneumoHose::setKeySymbolDisconnect(std::uint16_t key_symbol)
{
    ref_state.setKeySymbolDecrease(key_symbol);
}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
void PneumoHose::setKeyModifierDisconnect(std::uint16_t key_modifier)
{
    ref_state.setKeyModifierDecrease(key_modifier);
}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
void PneumoHose::setControl(std::set<uint16_t>* keys,
                                 control_signals_t* control_signals)
{
    Device::setControl(keys, control_signals);
    ref_state.setControl(pressed_keys);
}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
void PneumoHose::connect()
{
    ref_state.setPosition(2);

    // После вызова команды напрямую на один шаг отключаем управление с клавиатуры
    ref_state.setControl();
}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
void PneumoHose::disconnect()
{
    ref_state.setPosition(0);

    // После вызова команды напрямую на один шаг отключаем управление с клавиатуры
    ref_state.setControl();
}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
void PneumoHose::setExternalState(int state)
{
    ref_state.setSpringFirst(state != 0);
    ref_state.setSpringLast(state != 2);
    ref_state.setPosition(state);
}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
bool PneumoHose::isConnected() const
{
    return is_linked && (input_signals[HOSE_INPUT_IS_CONNECTED] == 1.0);
}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
void PneumoHose::setPressure(double value)
{
    output_signals[HOSE_OUTPUT_PIPE_PRESSURE] = value;
}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
void PneumoHose::setFlowCoeff(double value)
{
    output_signals[HOSE_OUTPUT_FLOW_COEFF] = value;
}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
void PneumoHose::setLength(double value)
{
    output_signals[HOSE_OUTPUT_LENGTH] = value;
}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
void PneumoHose::setRootCoord(dvec3 coord)
{
    output_signals[HOSE_OUTPUT_ROOT_COORD_X] = coord.x;
    output_signals[HOSE_OUTPUT_ROOT_COORD_Y] = coord.y;
    output_signals[HOSE_OUTPUT_ROOT_COORD_Z] = coord.z;
}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
void PneumoHose::setOrth(dvec3 vector)
{
    v_orth = vector;
}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
void PneumoHose::setRight(dvec3 vector)
{
    v_right = vector;
}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
void PneumoHose::setUp(dvec3 vector)
{
    v_up = vector;
}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
double PneumoHose::getFlow() const
{
    if (is_linked)
        return input_signals[HOSE_INPUT_FLOW_TO_PIPE];

    // Если нет соседнего рукава, то рукав открыт в атмосферу
    return -( output_signals[HOSE_OUTPUT_FLOW_COEFF]
            * output_signals[HOSE_OUTPUT_PIPE_PRESSURE] );
}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
double PneumoHose::getSideAngle() const
{
    return side_angle;
}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
double PneumoHose::getDownAngle() const
{
    return down_angle;
}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
dvec3 PneumoHose::getConnectionPoint() const
{
    return v_hose;
}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
sound_state_t PneumoHose::getSoundState(size_t idx) const
{
    if (idx == PIPE_DRAIN_FLOW_SOUND)
        return atm_flow_sound;
    if (idx == CONNECT_SOUND)
        return sound_state_t(isConnected());
    if (idx == DISCONNECT_SOUND)
        return sound_state_t(!isConnected());
    return sound_state_t();
}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
float PneumoHose::getSoundSignal(size_t idx) const
{
    if (idx == PIPE_DRAIN_FLOW_SOUND)
        return atm_flow_sound.createSoundSignal();
    if (idx == CONNECT_SOUND)
        return sound_state_t::createSoundSignal(isConnected());
    if (idx == DISCONNECT_SOUND)
        return sound_state_t::createSoundSignal(!isConnected());
    return sound_state_t::createSoundSignal(false);
}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
void PneumoHose::step(double t, double dt)
{
    ref_state.step(t, dt);
    output_signals[HOSE_OUTPUT_REF_STATE] = static_cast<double>(ref_state.getPosition()) - 1.0;

    // Возвращаем управление с клавиатуры, на случай если
    // оно было отключено прямыми вызовами connect()/disconnect()
    ref_state.setControl(pressed_keys);

    atm_flow_sound.state = !isConnected();
    atm_flow_sound.volume = K_sound * cbrt(getFlow());


    if (!isConnected())
    {
        // Если не соединены с соседним рукавом, считать углы не надо
        side_angle = 0.0;
        down_angle = 0.0;
        return;
    }

    // Вектор к точке соединения
    dvec3 v = { input_signals[HOSE_INPUT_CONNECTION_COORD_X] - output_signals[HOSE_OUTPUT_ROOT_COORD_X],
                input_signals[HOSE_INPUT_CONNECTION_COORD_Y] - output_signals[HOSE_OUTPUT_ROOT_COORD_Y],
                input_signals[HOSE_INPUT_CONNECTION_COORD_Z] - output_signals[HOSE_OUTPUT_ROOT_COORD_Z] };

    // Пересчитываем в локальные оси подвижной единицы
    v_hose =  { v_right.x * v.x + v_right.y * v.y + v_right.z * v.z,
                v_orth.x  * v.x + v_orth.y  * v.y + v_orth.z  * v.z,
                v_up.x    * v.x + v_up.y    * v.y + v_up.z    * v.z };
    const dvec3 v_unit = normalize(v_hose);
    side_angle = std::atan2(v_unit.x, v_unit.y);
    down_angle = std::asin(v_unit.z);
}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
void PneumoHose::ode_system(const state_vector_t &Y,
                           state_vector_t &dYdt,
                           double t)
{
    (void) Y;
    (void) dYdt;
    (void) t;
}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
void PneumoHose::load_config(CfgReader &cfg)
{
    QString secName = "Device";

    cfg.getString(secName, "Name", name);

    double tmp = 1.0;
    cfg.getDouble(secName, "FlowCoefficient", tmp);
    output_signals[HOSE_OUTPUT_FLOW_COEFF] = tmp;

    cfg.getDouble(secName, "K_sound", K_sound);

    tmp = 0.71;
    cfg.getDouble(secName, "Length", tmp);
    if (tmp > Physics::ZERO)
        output_signals[HOSE_OUTPUT_LENGTH] = tmp;
}
