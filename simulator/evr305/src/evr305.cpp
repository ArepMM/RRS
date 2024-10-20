#include    "evr305.h"

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
EVR305::EVR305(QObject *parent)
    : ElectroAirDistributor(parent)
    , V0(1.5e-3)
    , R(400.0)
    , L(0.5)
    , I_on(0.1)
{
    K.fill(0.0);
    A.fill(0.0);

    lines_num = EPB_LINES_NUM;
    setControlLinesNumber(lines_num);

    valves_num = VALVES_NUM;
    setValvesNumber(valves_num);

    zpk = new SwitchingValve(2e-3, 2e-3);
}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
EVR305::~EVR305()
{

}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
void EVR305::ode_system(const state_vector_t &Y,
                        state_vector_t &dYdt,
                        double t)
{
    Q_UNUSED(t)

    // Состояние отпускного вентиля
    double up = static_cast<double>(valve_state[RELEASE_VALVE]);
    // Состояние тормозного вентиля
    double ut = static_cast<double>(valve_state[BRAKE_VALVE]);

    // Поток воздуха из ЗР в РК (включены оба вентиля)
    double Q_sr_work = up * ut * cut(A[1] * (pSR - Y[WORK_PRESSURE]), 0.0, K[1]);
    // Поток воздуха из РК в атмосферу (отпускной вентиль выключен)
    double Q_work_atm = (1.0 - up) * cut(A[1] * (Y[WORK_PRESSURE]), 0.0, K[2]);

    // Давления на переключательном клапане
    double p1 = zpk->getPressure1();
    p_airdistBC = zpk->getPressure2();

    // Перемещение диафрагмы реле давления
    double s = A[2] * (Y[WORK_PRESSURE] - p1);

    // Поток воздуха на заполнение ТЦ
    double Q_sr_bc = cut(s, 0.0, K[3]) * (pSR - p1);
    // Поток воздуха на опорожнение ТЦ
    double Q_bc_atm = cut(-s, 0.0, K[4]) * p1;

    // Работа с ТЦ через переключательный клапан
    zpk->setInputFlow1(Q_sr_bc - Q_bc_atm);
    zpk->setInputFlow2(Q_airdistBC);

    zpk->setOutputPressure(pBC);
    QBC = zpk->getOutputFlow();

    // Поток в запасный резервуар
    QSR = Q_airdistSR - Q_sr_work - Q_sr_bc;
    p_airdistSR = pSR;

    // Поток в рабочую камеру
    dYdt[WORK_PRESSURE] = (Q_sr_work - Q_work_atm) / V0;
/*
//    QString(" pWORK ; pBC   ; pSR   ; zpk1  ; zpk2  ; SRwork ; SRbc   ; WORKat ; BCatm  ; U     ; f     ; I     ;R;B; u1b ; u2r ");
    DebugMsg = QString("%1;%2;%3;%4;%5;%6;%7;%8;%9;%10;%11;%12;%13;%14;%15;%16")
            .arg(Y[0], 7, 'f', 5)
            .arg(pBC, 7, 'f', 5)
            .arg(pSR, 7, 'f', 5)
            .arg(zpk->getPressure1(), 7, 'f', 5)
            .arg(zpk->getPressure2(), 7, 'f', 5)
            .arg(1000*Q_sr_work, 8, 'f', 5)
            .arg(1000*Q_sr_bc, 8, 'f', 5)
            .arg(1000*Q_work_atm, 8, 'f', 5)
            .arg(1000*Q_bc_atm, 8, 'f', 5)
            .arg(U[EPB_WORK_LINE], 7, 'f', 3)
            .arg(f[EPB_WORK_LINE], 7, 'f', 3)
            .arg(I[EPB_WORK_LINE], 7, 'f', 3)
            .arg(valve_state[RELEASE_VALVE])
            .arg(valve_state[BRAKE_VALVE])
            .arg(u1, 5, 'f', 3)
            .arg(u2, 5, 'f', 3);
*/
}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
void EVR305::preStep(state_vector_t &Y, double t)
{
    Q_UNUSED(Y)
    Q_UNUSED(t)

    // Ток, потребляемый катушкой вентиля
    double curr = U[EPB_WORK_LINE] / sqrt(R * R + pow(2.0 * Physics::PI * f[EPB_WORK_LINE] * L, 2.0));

    // Отпускной электромагнитный вентиль
    double I_release = curr;
    valve_state[RELEASE_VALVE] = (abs(I_release) > I_on);

    // Тормозной электромагнитный вентиль подключен через диод
    double I_brake = pf(curr);
    valve_state[BRAKE_VALVE] = (I_brake > I_on);

    // Ток, потребляемый электровоздухораспределителем
    I[EPB_WORK_LINE] = I_release + I_brake;
}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
void EVR305::load_config(CfgReader &cfg)
{
    QString secName = "Device";

    for (size_t i = 1; i < K.size(); ++i)
    {
        QString coeff = QString("K%1").arg(i);
        cfg.getDouble(secName, coeff, K[i]);
    }

    for (size_t i = 1; i < A.size(); ++i)
    {
        QString coeff = QString("A%1").arg(i);
        cfg.getDouble(secName, coeff, A[i]);
    }

    double tmp = 0.0;
    cfg.getDouble(secName, "V0", tmp);
    if (tmp > 1e-3)
        V0 = tmp;

    cfg.getDouble(secName, "R", R);
    cfg.getDouble(secName, "L", L);
    cfg.getDouble(secName, "I_on", I_on);

    zpk->read_config("zpk");
}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
void EVR305::stepKeysControl(double t, double dt)
{
    Q_UNUSED(t)
    Q_UNUSED(dt)
}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
void EVR305::step(double t, double dt)
{
    zpk->step(t, dt);

    ElectroAirDistributor::step(t, dt);
}

GET_ELECTRO_AIRDISTRIBUTOR(EVR305)
