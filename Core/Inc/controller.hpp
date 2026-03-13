#ifndef __CONTROLLER_HPP
#define __CONTROLLER_HPP

#include "z_main.h"
#include "component.hpp"
#include <algorithm>
#include <cmath>

class Controller {
public:
    enum Error {
        ERROR_NONE = 0,
        ERROR_INVALID_ESTIMATE = 1,
    };

    enum ControlMode {
        MODE_IDLE = 0,
        MODE_TORQUE_CONTROL = 1,
        MODE_VELOCITY_CONTROL = 2
    };

    struct Config_t
    {
        ControlMode control_mode = MODE_VELOCITY_CONTROL;
        float vel_gain = 0.3f;
        float vel_integrator_gain = 0.1f;
        float vel_limit = 50.0f;
        float vel_integrator_limit = INFINITY;
        bool enable_torque_mode_vel_limit = true;

        void set_control_mode(ControlMode mode) {
            control_mode = mode;
        }
    };

    Controller() = default;
    ~Controller() = default;

    bool update();
    void reset();

    float vel_integrator_torque_ = 0.0f;

    float vel_setpoint_ = 0.0f;
    float torque_setpoint_ = 0.0f;

    float input_vel_ = 0.0f;
    float input_torque_ = 0.0f;

    Config_t config_;
    Error error_ = Error::ERROR_NONE;

    Motor* motor_ = nullptr;

    InputPort<float> vel_estimate_src_;

    OutputPort<float> torque_out_;
};

// this is technically not thread-safe but practically it might be
inline Controller::Error operator | (Controller::Error a, Controller::Error b) { return static_cast<Controller::Error>(static_cast<std::underlying_type_t<Controller::Error>>(a) | static_cast<std::underlying_type_t<Controller::Error>>(b)); }
inline Controller::Error operator & (Controller::Error a, Controller::Error b) { return static_cast<Controller::Error>(static_cast<std::underlying_type_t<Controller::Error>>(a) & static_cast<std::underlying_type_t<Controller::Error>>(b)); }
inline Controller::Error operator ^ (Controller::Error a, Controller::Error b) { return static_cast<Controller::Error>(static_cast<std::underlying_type_t<Controller::Error>>(a) ^ static_cast<std::underlying_type_t<Controller::Error>>(b)); }
inline Controller::Error& operator |= (Controller::Error &a, Controller::Error b) { return reinterpret_cast<Controller::Error&>(reinterpret_cast<std::underlying_type_t<Controller::Error>&>(a) |= static_cast<std::underlying_type_t<Controller::Error>>(b)); }
inline Controller::Error& operator &= (Controller::Error &a, Controller::Error b) { return reinterpret_cast<Controller::Error&>(reinterpret_cast<std::underlying_type_t<Controller::Error>&>(a) &= static_cast<std::underlying_type_t<Controller::Error>>(b)); }
inline Controller::Error& operator ^= (Controller::Error &a, Controller::Error b) { return reinterpret_cast<Controller::Error&>(reinterpret_cast<std::underlying_type_t<Controller::Error>&>(a) ^= static_cast<std::underlying_type_t<Controller::Error>>(b)); }
inline Controller::Error operator ~ (Controller::Error a) { return static_cast<Controller::Error>(~static_cast<std::underlying_type_t<Controller::Error>>(a)); }



#endif // __CONTROLLER_HPP