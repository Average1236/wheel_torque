#ifndef __MOTOR_HPP
#define __MOTOR_HPP

#include "z_main.h"
#include "component.hpp"
#include <algorithm>
#include <cmath>
#include "stm32f4xx_hal_gpio.h"

class Motor {
public:
    struct Config_t
    {
        float current_limit = 3.3f;
        float current_p_gain=0.3f;
        float current_i_gain=0.1f;
        float shunt_conductance = 1.0f / SHUNT_RESISTANCE;
        float dc_calib_tau = 0.2f;
        float max_dc_calib_ = 1.5f;

        float direction = 1.0f; // 1 or -1, this is used to correct the direction of the motor without having to change the wiring

        float torque_constant = 0.1f; // Nm/A
    };
    
    enum Error {
        ERROR_NONE = 0,
        ERROR_OVERCURRENT = 1,
        ERROR_CURRENT_SENSE_SATURATION = 2,
        ERROR_UNKNOWN_CURRENT_MEASUREMENT = 4,
        ERROR_UNKNOWN_TORQUE = 8,
        ERROR_UNKNOWN_VELOCITY = 16,
    };

    Motor(TIM_HandleTypeDef* timer, uint16_t timer_channel) : timer_(timer), timer_channel_(timer_channel) {};
    ~Motor() = default;

    std::optional<float> current_from_adc(uint32_t adc_val);
    void current_meas_cb(std::optional<float> current);
    void dc_calib_cb(std::optional<float> current);
    float max_available_torque();
    void apply_pwm_timings(uint16_t timing, float torque_dir, bool tentative);
    void update();
    void arm();
    void disarm_with_error(Error error);
    void disarm();

    bool is_armed_ = false;
    int armed_state_ = 0;
    TIM_HandleTypeDef* timer_;
    uint32_t timer_channel_ = 0;
    float current_meas_ = 0.0f;
    float dc_calib_running_since_ = 0.0f;
    float dc_calib_ = 0.0f;
    Config_t config_;
    Error error_ = Error::ERROR_NONE;
    float current_integral_ = 0.0f; // this is the integral term of the current controller, it is stored here to be used in the next update

    InputPort<float> torque_setpoint_src_;
    InputPort<float> vel_estimate_src_;
};

// this is technically not thread-safe but practically it might be
inline Motor::Error operator | (Motor::Error a, Motor::Error b) { return static_cast<Motor::Error>(static_cast<std::underlying_type_t<Motor::Error>>(a) | static_cast<std::underlying_type_t<Motor::Error>>(b)); }
inline Motor::Error operator & (Motor::Error a, Motor::Error b) { return static_cast<Motor::Error>(static_cast<std::underlying_type_t<Motor::Error>>(a) & static_cast<std::underlying_type_t<Motor::Error>>(b)); }
inline Motor::Error operator ^ (Motor::Error a, Motor::Error b) { return static_cast<Motor::Error>(static_cast<std::underlying_type_t<Motor::Error>>(a) ^ static_cast<std::underlying_type_t<Motor::Error>>(b)); }
inline Motor::Error& operator |= (Motor::Error &a, Motor::Error b) { return reinterpret_cast<Motor::Error&>(reinterpret_cast<std::underlying_type_t<Motor::Error>&>(a) |= static_cast<std::underlying_type_t<Motor::Error>>(b)); }
inline Motor::Error& operator &= (Motor::Error &a, Motor::Error b) { return reinterpret_cast<Motor::Error&>(reinterpret_cast<std::underlying_type_t<Motor::Error>&>(a) &= static_cast<std::underlying_type_t<Motor::Error>>(b)); }
inline Motor::Error& operator ^= (Motor::Error &a, Motor::Error b) { return reinterpret_cast<Motor::Error&>(reinterpret_cast<std::underlying_type_t<Motor::Error>&>(a) ^= static_cast<std::underlying_type_t<Motor::Error>>(b)); }
inline Motor::Error operator ~ (Motor::Error a) { return static_cast<Motor::Error>(~static_cast<std::underlying_type_t<Motor::Error>>(a)); }



#endif /* __MOTOR_HPP */
