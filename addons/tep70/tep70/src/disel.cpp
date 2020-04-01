#include    "disel.h"

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
Disel::Disel(QObject *parent) : Device(parent)
  , V_oil(1.5)
  , Q_emn(0.0)
{
    std::fill(K.begin(), K.end(), 0.0);

    K[0] = 15.0;
}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
Disel::~Disel()
{

}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
void Disel::setQ_emn(double Q_emn)
{
    this->Q_emn = Q_emn;
}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
void Disel::preStep(state_vector_t &Y, double t)
{
    Q_UNUSED(t)
}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
void Disel::ode_system(const state_vector_t &Y, state_vector_t &dYdt, double t)
{
    Q_UNUSED(t)

    // Давление масла в системе смазки
    dYdt[0] = (Q_emn + K[1] * Y[1] - K[0] * Y[0]) / V_oil;
}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
void Disel::load_config(CfgReader &cfg)
{
    QString secName = "Device";

    cfg.getDouble(secName, "V_oil", V_oil);

    for (size_t i = 0; i < K.size(); ++i)
    {
        QString param = QString("K%1").arg(i);
        cfg.getDouble(secName, param, K[i]);
    }
}