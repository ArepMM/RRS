#include    <connector.h>
#include    <Journal.h>

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
Connector::Connector(QObject *parent) : QObject(parent)
{

}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
Connector::~Connector()
{

}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
void Connector::configure(CfgReader &cfg,
                          QDomNode secNode,
                          traj_list_t &traj_list)
{
    Q_UNUSED(traj_list)

    cfg.getString(secNode, "Name", name);
    cfg.getInt(secNode, "state", state);

    Journal::instance()->info("Connector " + name + " will be initialized...");
}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
void Connector::step(double t, double dt)
{
    for (auto conn_device : devices)
    {
        conn_device->step(t, dt);
    }
}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
QByteArray Connector::serialize()
{
    QBuffer data;

    return data.data();
}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
void Connector::deserialize(QByteArray &data, traj_list_t &traj_list)
{

}
