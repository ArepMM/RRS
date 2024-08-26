#ifndef     SWITCH_H
#define     SWITCH_H

#include    <connector.h>

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
class TOPOLOGY_EXPORT Switch : public Connector
{
    Q_OBJECT

public:

    Switch(QObject *parent = Q_NULLPTR);

    ~Switch();

    Trajectory *getFwdTraj() const override;

    Trajectory *getBwdTraj() const override;

    void configure(CfgReader &cfg, QDomNode secNode, traj_list_t &traj_list) override;

    QByteArray serialize() override;

    void deserialize(QByteArray &data, traj_list_t &traj_list) override;

    int getStateFwd() const
    {
        return state_fwd;
    }

    int getStateBwd() const
    {
        return state_bwd;
    }

    void setStateFwd(int state_fwd)
    {
        this->state_fwd = state_fwd;
    }

    void setStateBwd(int state_bwd)
    {
        this->state_bwd = state_bwd;
    }

private:

    Trajectory *fwdMinusTraj = Q_NULLPTR;

    Trajectory *fwdPlusTraj = Q_NULLPTR;

    Trajectory *bwdMinusTraj = Q_NULLPTR;

    Trajectory *bwdPlusTraj = Q_NULLPTR;

    /// Состояние стрелки впереди
    int state_fwd = 1;

    /// Состояние стрелки сзади
    int state_bwd = 1;

    void serialize_connected_trajectory(QDataStream &stream, Trajectory *traj);

    Trajectory *deserialize_connected_trajectory(QDataStream &stream,
                                          traj_list_t &traj_list);
};

#endif // SWITCH_H
