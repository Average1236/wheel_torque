#ifndef __MOTOR_HPP
#define __MOTOR_HPP

#include "z_main.h"
#include "component.hpp"
#include <algorithm>
#include <cmath>
#include "stm32f4xx_hal_gpio.h"

class ButterworthFilter {
public:
    ButterworthFilter() = default;
    
    // Initialize the 2nd order Butterworth filter coefficients
    void init(float sample_freq, float cutoff_freq) {
        float wc = 2.0f * 3.14159265359f * cutoff_freq;
        float c = 1.0f / std::tan(wc / (2.0f * sample_freq));
        
        float den = 1.0f + 1.41421356237f * c + c * c;
        
        b0 = 1.0f / den;
        b1 = 2.0f * b0;
        b2 = b0;
        a1 = 2.0f * (1.0f - c * c) / den;
        a2 = (1.0f - 1.41421356237f * c + c * c) / den;
        
        reset();
    }
    
    float process(float input) {
        float output = b0 * input + b1 * x1 + b2 * x2 - a1 * y1 - a2 * y2;
        
        x2 = x1;
        x1 = input;
        y2 = y1;
        y1 = output;
        
        return output;
    }
    
    void reset() {
        x1 = x2 = y1 = y2 = 0.0f;
    }

private:
    float b0 = 1.0f, b1 = 0.0f, b2 = 0.0f;
    float a1 = 0.0f, a2 = 0.0f;
    float x1 = 0.0f, x2 = 0.0f;
    float y1 = 0.0f, y2 = 0.0f;
};

class Motor {

public:
    struct Config_t
    {
        float current_limit = 5.3f; // rated current + some margin [A]
        float current_p_gain = 1.0f;
        float current_i_gain=50.0f;
        float shunt_conductance = 1.0f / SHUNT_RESISTANCE;
        float dc_calib_tau = 0.2f;
        float max_dc_calib_ = 1.5f;

        // float direction = -1.0f; // 1 or -1, this is used to correct the direction of the motor without having to change the wiring

        float torque_constant = 0.0355f; // Nm/A
        float current_filter_cutoff = 10000.0f; // Hz, Butterworth filter cutoff freq
    };
    
    enum Error {
        ERROR_NONE =                          0x00000000,
        ERROR_OVERCURRENT =                   0x00000001,
        ERROR_CURRENT_SENSE_SATURATION =      0x00000002,
        ERROR_UNKNOWN_CURRENT_MEASUREMENT =   0x00000004,
        ERROR_UNKNOWN_TORQUE =                0x00000008,
        ERROR_UNKNOWN_VELOCITY =              0x00000010,
        ERROR_TIMER_UPDATE_MISSED =           0x00000020,
        ERROR_BAD_TIMING =                    0x00000040,
        ERROR_CONTROL_DEADLINE_MISSED =       0x00000080,
    };

    Motor(TIM_HandleTypeDef* timer, uint16_t timer_channel) : timer_(timer), timer_channel_(timer_channel) {};
    ~Motor() = default;

    std::optional<float> current_from_adc(uint32_t adc_val);
    void current_meas_cb(std::optional<float> current);
    void dc_calib_cb(std::optional<float> current);
    void pwm_update_cb();
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
    std::optional<float> current_meas_ = std::nullopt;
    ButterworthFilter current_filter_;
    float dc_calib_running_since_ = 0.0f;
    float dc_calib_ = 0.0f;
    bool dc_calib_valid_ = false;
    Config_t config_;
    Error error_ = Error::ERROR_NONE;
    float current_integral_ = 0.0f; // this is the integral term of the current controller, it is stored here to be used in the next update
    float current_setpoint_ = 0.0f;
    float torque_direction_ = 1.0f; // this is the direction of the torque that we are currently applying, it is used to make sure the correct direction is applied when we are in torque control mode and the velocity changes sign
    float last_torque_direction_ = 1.0f; // this is the direction of the torque that we applied in the last update, it is used to detect changes in the velocity direction when we are in torque control mode

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
