#include    "brake-crane.h"

#include    <QLibrary>

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
BrakeCrane::BrakeCrane(QObject *parent) : BrakeDevice (parent)
  , is_hold(false)
  , is_brake(false)
  , p0(0.5)
  , Ver(1.0)
  , pFL(0.0)
  , pBP(0.0)
  , QFL(0.0)
  , QBP(0.0)
  , Qer(0.0)
{
    sounds.resize(NUM_SOUNDS, sound_state_t());
}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
BrakeCrane::~BrakeCrane()
{

}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
bool BrakeCrane::isHold() const
{
    return is_hold;
}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
bool BrakeCrane::isBrake() const
{
    return is_brake;
}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
void BrakeCrane::setChargePressure(double value)
{
    p0 = value;
}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
void BrakeCrane::setFLpressure(double value)
{
    pFL = value;
}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
double BrakeCrane::getFLflow() const
{
    return QFL;
}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
void BrakeCrane::setBPpressure(double value)
{
    pBP = value;
}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
double BrakeCrane::getBPflow() const
{
    return QBP;
}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
void BrakeCrane::setERflow(double Qer)
{
    this->Qer = Qer;
}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
double BrakeCrane::getERpressure() const
{
    return getY(ER_PRESSURE);
}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
sound_state_t BrakeCrane::getSoundState(size_t idx) const
{
    if (idx < sounds.size())
        return sounds[idx];
    return Device::getSoundState();
}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
float BrakeCrane::getSoundSignal(size_t idx) const
{
    if (idx < sounds.size())
        return sounds[idx].createSoundSignal();
    return Device::getSoundSignal();
}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
void BrakeCrane::ode_system(const state_vector_t &Y,
                            state_vector_t &dYdt,
                            double t)
{
    Q_UNUSED(Y)
    Q_UNUSED(t)

    dYdt[ER_PRESSURE] = Qer / Ver;
}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
BrakeCrane *loadBrakeCrane(QString lib_path)
{
    BrakeCrane *crane = nullptr;

    QLibrary lib(lib_path);

    if (lib.load())
    {
        GetBrakeCrane getBrakeCrane = reinterpret_cast<GetBrakeCrane>(lib.resolve("getBrakeCrane"));

        if (getBrakeCrane)
        {
            crane = getBrakeCrane();
        }
    }

    return crane;
}
