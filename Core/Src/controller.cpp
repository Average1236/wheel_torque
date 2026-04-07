#include "controller.hpp"

// Debug
volatile float torque_out_debug = 0;

void Controller::reset() {
    vel_setpoint_ = 0.0f;
    torque_setpoint_ = 0.0f;
    vel_integrator_torque_ = 0.0f;
    error_ = Error::ERROR_NONE;
}

static float limitVel(const float vel_limit, const float vel_estimate, const float vel_gain, const float torque) {
    float Tmax = (vel_limit - vel_estimate) * vel_gain;
    float Tmin = (-vel_limit - vel_estimate) * vel_gain;
    return std::clamp(torque, Tmin, Tmax);
}

bool Controller::update() {
    // std::optional<float> vel_estimate = vel_estimate_src_.present();
    std::optional<float> vel_estimate = 0.0f; // TODO: remove this when the encoder is working

    vel_setpoint_ = std::clamp(input_vel_, -config_.vel_limit, config_.vel_limit);
    float torque_limit = motor_->max_available_torque();
    torque_setpoint_ = std::clamp(input_torque_, -torque_limit, torque_limit);

    float torque = torque_setpoint_;
    float vel_err = 0.0f;
    if (config_.control_mode >= MODE_VELOCITY_CONTROL) {
        if (!vel_estimate.has_value()) {
            error_ |= Error::ERROR_INVALID_ESTIMATE;
            return false;
        }
        vel_err = vel_setpoint_ - vel_estimate.value();
        torque = config_.vel_gain * vel_err + vel_integrator_torque_;
    }

    // Velocity limiting in current mode
    if (config_.control_mode < MODE_VELOCITY_CONTROL && config_.enable_torque_mode_vel_limit) {
        if (!vel_estimate.has_value()) {
            error_ |= Error::ERROR_INVALID_ESTIMATE;
            return false;
        }
        torque = limitVel(config_.vel_limit, *vel_estimate, config_.vel_gain, torque);
    }

    // Torque limiting
    bool limited = false;
    if (torque > torque_limit) {
        limited = true;
        torque = torque_limit;
    }
    if (torque < -torque_limit) {
        limited = true;
        torque = -torque_limit;
    }

    // Velocity integrator (behaviour dependent on limiting)
    if (config_.control_mode < MODE_VELOCITY_CONTROL) {
        // reset integral if not in use
        vel_integrator_torque_ = 0.0f;
    } else {
        if (limited) {
            // TODO make decayfactor configurable
            vel_integrator_torque_ *= 0.99f;
        } else {
            vel_integrator_torque_ += (config_.vel_integrator_gain * CURRENT_MEAS_PERIOD_S) * vel_err;
        }
        // integrator limiting to prevent windup 
        vel_integrator_torque_ = std::clamp(vel_integrator_torque_, -config_.vel_integrator_limit, config_.vel_integrator_limit);
    }

    torque_out_ = torque;

    // Debug
    torque_out_debug = torque;

    return true;
}