#ifndef     PNEUMO_HOSE_H
#define     PNEUMO_HOSE_H

#include    "device.h"
#include    "vec3.h"

#include    "pneumo-hose-data.h"

//------------------------------------------------------------------------------
// Рукав пневматической магистрали
//------------------------------------------------------------------------------
class DEVICE_EXPORT PneumoHose : public Device
{
public:

    /// Конструктор
    PneumoHose(QObject *parent = nullptr);

    /// Деструктор
    ~PneumoHose();

    /// Задать управляющую клавишу для команды соединить рукава
    void setKeySymbolConnect(std::uint16_t key_symbol);

    /// Задать клавишу-модификатор для команды соединить рукава
    void setKeyModifierConnect(std::uint16_t key_modifier);

    /// Задать управляющую клавишу для команды разъединить рукава
    void setKeySymbolDisconnect(std::uint16_t key_symbol);

    /// Задать клавишу-модификатор для команды разъединить рукава
    void setKeyModifierDisconnect(std::uint16_t key_modifier);

    void setControl(std::set<uint16_t>* keys = nullptr,
                    control_signals_t* control_signals = nullptr) override;

    /// Соединить рукава
    void connect();

    /// Установить состояние рукава через IOController (0 — разъединить, 1 — нейтраль, 2 — соединить)
    void setExternalState(int state);

    /// Разъединить рукава
    void disconnect();

    /// Состояние соединения рукавов
    bool isConnected() const;

    /// Задать давление в рукаве
    void setPressure(double value);

    /// Задать коэффициент перетока через рукав
    void setFlowCoeff(double value);

    /// Задать длину рукава, м
    void setLength(double value);

    /// Задать точку крепления рукава в мировых координатах, м
    void setRootCoord(dvec3 coord);

    /// Задать вектор продольной оси подвижной единицы, м
    void setOrth(dvec3 vector);

    /// Задать вектор поперечной оси подвижной единицы, м
    void setRight(dvec3 vector);

    /// Задать вектор вертикальной оси подвижной единицы, м
    void setUp(dvec3 vector);

    /// Получить поток через рукав
    double getFlow() const;

    /// Получить угол отклонения рукава от продольной оси
    /// (<0 - повёрнут влево, >0 - повёрнут вправо), радиан
    double getSideAngle() const;

    /// Получить угол свешивания рукава вниз от продольной оси, радиан
    double getDownAngle() const;

    dvec3 getConnectionPoint() const;

    virtual void step(double t, double dt) override;

    enum {
        NUM_SOUNDS = 3,
        PIPE_DRAIN_FLOW_SOUND = 0, ///< Звук опорожнения магистрали через рукав
        CONNECT_SOUND = 1,   ///< Звук соединения рукавов
        DISCONNECT_SOUND = 2 ///< Звук рассоединения рукавов
    };
    /// Состояние звука
    virtual sound_state_t getSoundState(size_t idx = PIPE_DRAIN_FLOW_SOUND) const override;

    /// Сигнал состояния звука
    virtual float getSoundSignal(size_t idx = PIPE_DRAIN_FLOW_SOUND) const override;

protected:

    /// Коэффициент громкости озвучки к потоку опорожнения магистрали через рукав
    double K_sound = 1.5;

    /// Управление рукавом с клавиатуры: 0 - рассоединить, 1 - оставить текущее состояние, 2 - соединить
    SwitcherControl ref_state = SwitcherControl(3);

    /// Звук опорожнения магистрали через рукав
    sound_state_t atm_flow_sound = sound_state_t();

    /// Вектор продольной оси подвижной единицы
    dvec3 v_orth = {0.0, 1.0, 0.0};

    /// Вектор поперечной оси подвижной единицы
    dvec3 v_right = {1.0, 0.0, 0.0};

    /// Вектор вертикальной оси подвижной единицы
    dvec3 v_up = {0.0, 0.0, 1.0};

    /// Вектор от точки крепления рукава к точке соединения с соседним рукавом
    dvec3 v_hose = {0.0, 0.0, 0.0};

    /// Угол отклонения рукава от продольной оси
    /// (<0 - повёрнут влево, >0 - повёрнут вправо), радиан
    double side_angle = 0.0;

    /// Угол свешивания рукава вниз от продольной оси, радиан
    double down_angle = 0.0;

    virtual void ode_system(const state_vector_t &Y,
                            state_vector_t &dYdt,
                            double t) override;

    /// Загрузка параметров из конфигурационного файла
    virtual void load_config(CfgReader &cfg) override;
};

#endif // PNEUMO_HOSE_H
