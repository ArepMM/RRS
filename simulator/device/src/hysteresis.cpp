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
    , state(init_state)
{

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
        state = false;

    if (value >= max)
        state = true;
}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
bool Hysteresis::getState() const
{
    return state;
}