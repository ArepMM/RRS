#ifndef     KME_60_044_H
#define     KME_60_044_H

#include    "traction-controller.h"
#include    "km-state.h"
#include    "timer.h"

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
class ControllerKME_60_044 : public TractionController
{
public:

    ControllerKME_60_044(QObject *parent = Q_NULLPTR);

    ~ControllerKME_60_044();

    km_state_t getState() const;

    float getMainHandlePos() const;

    float getReversHandlePos() const;

    void setReversPos(int pos);

    enum {
        NUM_SOUNDS = 2,
        REVERS_CHANGE_POS_SOUND = 0,    ///< Звук переключения реверсора
        MAIN_CHANGE_POS_SOUND = 1       ///< Звук переключения контроллера
    };
    /// Состояние звука
    sound_state_t getSoundState(size_t idx = REVERS_CHANGE_POS_SOUND) const;

    /// Сигнал состояния звука
    float getSoundSignal(size_t idx = REVERS_CHANGE_POS_SOUND) const;

private:

    /// Состояние главного вала
    int         main_pos;
    /// Состояние реверсивного вала
    int         revers_pos;

    /// Положение главной рукоятки
    float       main_handle_pos;

    /// Положение реверсивной рукоятки
    float       revers_handle_pos;

    /// Состояние контроллера
    km_state_t  state;

    /// Счётчик и состояние звуков
    std::array <sound_state_t, NUM_SOUNDS> sounds;

    enum
    {
        SWITCH_TIMEOUT = 300
    };

    Timer       *incMainPos;
    Timer       *decMainPos;

    Timer       *incReversPos;
    Timer       *decReversPos;

    void preStep(state_vector_t &Y, double t);

    void ode_system(const state_vector_t &Y, state_vector_t &dYdt, double t);

    void load_config(CfgReader &cfg);

    void stepKeysControl(double t, double dt);

    void soundMainChangePos();
    void soundReversChangePos();

private slots:

    void incMain();

    void decMain();

    void incRevers();

    void decRevers();
};

#endif // KME_60_044_H
