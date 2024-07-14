#include    "hysteresis.h"

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
Hysteresis::Hysteresis(double min_value,
                       double max_value,
                       bool init_state,
                       QObject *parent)
    : QObject(parent)
    , min(min_value)
    , max(max_value)
{
    if (init_state)
        state.set();
    else
        state.reset();
}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
Hysteresis::~Hysteresis()
{

}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
void Hysteresis::setRange(double min_value, double max_value)
{
    min = min_value;
    max = max_value;
}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
void Hysteresis::setValue(double value)
{
    if (value <= min)
        state.reset();

    if (value >= max)
        state.set();
}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
bool Hysteresis::getState() const
{
    return state.getState();
}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
sound_state_t Hysteresis::getSoundOn() const
{
    return state.getSoundOn();
}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
sound_state_t Hysteresis::getSoundOff() const
{
    return state.getSoundOff();
}
