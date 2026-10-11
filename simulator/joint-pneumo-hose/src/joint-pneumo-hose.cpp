#include    "joint-pneumo-hose.h"

#include    "CfgReader.h"
#include    "physics.h"
#include    "vec3.h"

#include    "device.h"
#include    "core/get_module.h"

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
JointPneumoHose::JointPneumoHose() : Joint()
  , is_connected(false)
{
    devices.resize(NUM_CONNECTORS);
}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
JointPneumoHose::~JointPneumoHose()
{

}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
void JointPneumoHose::step(double t, double dt)
{
    Q_UNUSED(t)
    Q_UNUSED(dt)

    stepConnectionCheck();
    stepFlowCalc();
}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
void JointPneumoHose::stepConnectionCheck()
{
    // Управление соединением рукавов
    if (   (devices[FWD]->getOutputSignal(HOSE_OUTPUT_REF_STATE) == 1.0)
        || (devices[BWD]->getOutputSignal(HOSE_OUTPUT_REF_STATE) == 1.0) )
    {
        is_connected = true;
    }

    // Управление разъединением рукавов
    if (   (devices[FWD]->getOutputSignal(HOSE_OUTPUT_REF_STATE) == -1.0)
        || (devices[BWD]->getOutputSignal(HOSE_OUTPUT_REF_STATE) == -1.0) )
    {
        is_connected = false;
    }

    if (!is_connected)
    {
        devices[FWD]->setInputSignal(HOSE_INPUT_CONNECTION_COORD_X, 0.0);
        devices[FWD]->setInputSignal(HOSE_INPUT_CONNECTION_COORD_Y, 0.0);
        devices[FWD]->setInputSignal(HOSE_INPUT_CONNECTION_COORD_Z, 0.0);
        devices[BWD]->setInputSignal(HOSE_INPUT_CONNECTION_COORD_X, 0.0);
        devices[BWD]->setInputSignal(HOSE_INPUT_CONNECTION_COORD_Y, 0.0);
        devices[BWD]->setInputSignal(HOSE_INPUT_CONNECTION_COORD_Z, 0.0);
        return;
    }

    // Расчёт геометрии рукавов
    const double len_fwd = devices[FWD]->getOutputSignal(HOSE_OUTPUT_LENGTH);
    const dvec3 coord_fwd = { devices[FWD]->getOutputSignal(HOSE_OUTPUT_ROOT_COORD_X),
                              devices[FWD]->getOutputSignal(HOSE_OUTPUT_ROOT_COORD_Y),
                              devices[FWD]->getOutputSignal(HOSE_OUTPUT_ROOT_COORD_Z) };
    const double len_bwd = devices[BWD]->getOutputSignal(HOSE_OUTPUT_LENGTH);
    const dvec3 coord_bwd = { devices[BWD]->getOutputSignal(HOSE_OUTPUT_ROOT_COORD_X),
                              devices[BWD]->getOutputSignal(HOSE_OUTPUT_ROOT_COORD_Y),
                              devices[BWD]->getOutputSignal(HOSE_OUTPUT_ROOT_COORD_Z) };

    // Расстояния между точками крепления
    const dvec3 coord_delta = coord_bwd - coord_fwd;
    const double len2 = length2(coord_delta);
    const double len = std::sqrt(len2);

    // Неравенства треугольника
    if (   ((len_fwd + len_bwd - len) < Physics::ZERO)
        || ((len + len_fwd - len_bwd) < Physics::ZERO)
        || ((len_bwd + len - len_fwd) < Physics::ZERO))
    {
        is_connected = false;
        devices[FWD]->setInputSignal(HOSE_INPUT_CONNECTION_COORD_X, 0.0);
        devices[FWD]->setInputSignal(HOSE_INPUT_CONNECTION_COORD_Y, 0.0);
        devices[FWD]->setInputSignal(HOSE_INPUT_CONNECTION_COORD_Z, 0.0);
        devices[BWD]->setInputSignal(HOSE_INPUT_CONNECTION_COORD_X, 0.0);
        devices[BWD]->setInputSignal(HOSE_INPUT_CONNECTION_COORD_Y, 0.0);
        devices[BWD]->setInputSignal(HOSE_INPUT_CONNECTION_COORD_Z, 0.0);
        return;
    }

    // Единичный вектор между точками крепления
    const dvec3 v_unit = coord_delta / len;
    // Единичный вектор вниз
    const dvec3 v_down = {0.0, 0.0, -1.0};
    // Единичный вектор вдоль высоты треугольника в направлении вниз
    const dvec3 v_h = normalize(v_down - v_unit * dot(v_down, v_unit));

    // Проекция одного из рукавов
    const double fwd_proj =   (len2 + len_fwd * len_fwd - len_bwd * len_bwd)
                            / (2.0 * len);
    // Высота треугольника
    const double h = std::sqrt(len_fwd * len_fwd - fwd_proj * fwd_proj);

    // Рассчитываем точку соединения рукавов
    const dvec3 point = coord_fwd + v_unit * fwd_proj + v_h * h;

    devices[FWD]->setInputSignal(HOSE_INPUT_CONNECTION_COORD_X, point.x);
    devices[FWD]->setInputSignal(HOSE_INPUT_CONNECTION_COORD_Y, point.y);
    devices[FWD]->setInputSignal(HOSE_INPUT_CONNECTION_COORD_Z, point.z);
    devices[BWD]->setInputSignal(HOSE_INPUT_CONNECTION_COORD_X, point.x);
    devices[BWD]->setInputSignal(HOSE_INPUT_CONNECTION_COORD_Y, point.y);
    devices[BWD]->setInputSignal(HOSE_INPUT_CONNECTION_COORD_Z, point.z);
}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
void JointPneumoHose::stepFlowCalc()
{
    // Расчёт потоков через рукава
    double p_fwd = devices[FWD]->getOutputSignal(HOSE_OUTPUT_PIPE_PRESSURE);
    double p_bwd = devices[BWD]->getOutputSignal(HOSE_OUTPUT_PIPE_PRESSURE);

    double flow_coeff_fwd = devices[FWD]->getOutputSignal(HOSE_OUTPUT_FLOW_COEFF);
    double flow_coeff_bwd = devices[BWD]->getOutputSignal(HOSE_OUTPUT_FLOW_COEFF);

    if (is_connected)
    {
        devices[FWD]->setInputSignal(HOSE_INPUT_IS_CONNECTED, 1.0);
        devices[BWD]->setInputSignal(HOSE_INPUT_IS_CONNECTED, 1.0);

        // Поток через соединённые рукава
        double flow_coeff = min(flow_coeff_fwd, flow_coeff_bwd);
        double flow = flow_coeff * (p_fwd - p_bwd);

        devices[FWD]->setInputSignal(HOSE_INPUT_FLOW_TO_PIPE, -flow);
        devices[BWD]->setInputSignal(HOSE_INPUT_FLOW_TO_PIPE, flow);
    }
    else
    {
        devices[FWD]->setInputSignal(HOSE_INPUT_IS_CONNECTED, 0.0);
        devices[BWD]->setInputSignal(HOSE_INPUT_IS_CONNECTED, 0.0);

        // Поток из разъединённых рукавов в атмосферу
        double flow = flow_coeff_fwd * p_fwd;
        devices[FWD]->setInputSignal(HOSE_INPUT_FLOW_TO_PIPE, -flow);

        flow = flow_coeff_bwd * p_bwd;
        devices[BWD]->setInputSignal(HOSE_INPUT_FLOW_TO_PIPE, -flow);
    }
}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
void JointPneumoHose::load_config(CfgReader &cfg)
{
    QString secName = "Joint";
    cfg.getBool(secName, "initConnectionState", is_connected);
}

GET_MODULE(JointPneumoHose)
