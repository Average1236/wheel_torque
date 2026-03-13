#include "motor.hpp"

static constexpr auto CURRENT_ADC_LOWER_BOUND = (uint32_t)((float)(1 << 12) * CURRENT_SENSE_MIN_VOLT / 3.3f);
static constexpr auto CURRENT_ADC_UPPER_BOUND = (uint32_t)((float)(1 << 12) * CURRENT_SENSE_MAX_VOLT / 3.3f);

std::optional<float> Motor::current_from_adc(uint32_t adc_val) {
  // Make sure the measurements don't come too close to the current sensor's hardware limitations
  if (adc_val < CURRENT_ADC_LOWER_BOUND || adc_val > CURRENT_ADC_UPPER_BOUND) {
    error_ |= Error::ERROR_CURRENT_SENSE_SATURATION;
    return std::nullopt;
  }

  float amp_out_volt = (3.3f / (float)(1 << 12)) * (float)adc_val;
  float shunt_volt = amp_out_volt / CURRENT_MEAS_GAIN;
  float current = shunt_volt * config_.shunt_conductance;
  return current;
}

void Motor::current_meas_cb(std::optional<float> current) {
    if (armed_state_ == 1 || armed_state_ == 2) {
        current_meas_ = 0.0f; // this is needed to make sure the motor doesn't get a large current setpoint at the moment of arming due to some stale value in current_meas_
        armed_state_++;
        return;
    }
    bool dc_calib_valid = (dc_calib_running_since_ >= config_.dc_calib_tau * 7.5f) && (abs(dc_calib_) < config_.max_dc_calib_);
    if (current.has_value() && dc_calib_valid) {
        float current_val = current.value() - dc_calib_;
        if (current_val > config_.current_limit || current_val < -config_.current_limit) {
        disarm_with_error(Error::ERROR_OVERCURRENT);
        } else {
        current_meas_ = current_val;
        }
    } else if (is_armed_) {
        disarm_with_error(Error::ERROR_UNKNOWN_CURRENT_MEASUREMENT);
    }
}

void Motor::disarm() {
    CRITICAL_SECTION() {
        is_armed_ = false;
        armed_state_ = 0;
        TIM_HandleTypeDef* timer = timer_;
        timer->Instance->BDTR &= ~TIM_BDTR_AOE; // prevent the PWMs from automatically enabling at the next update
        __HAL_TIM_MOE_DISABLE_UNCONDITIONALLY(timer);
    }
}

void Motor::disarm_with_error(Error error) {
    error_ |= error;
    disarm();
}

/**
 * @brief Arms the PWM outputs that belong to this motor.
 *
 * Note that this does not activate the PWM outputs immediately, it just sets
 * a flag so they will be enabled later.
 * 
 * The sequence goes like this:
 *  - Motor::arm() sets the is_armed_ flag.
 *  - On the next timer update event Motor::timer_update_cb() gets called in an
 *    interrupt context
 *  - Motor::timer_update_cb() runs specified control law to determine PWM values
 *  - Motor::timer_update_cb() calls Motor::apply_pwm_timings()
 *  - Motor::apply_pwm_timings() sets the output compare registers and the AOE
 *    (automatic output enable) bit.
 *  - On the next update event the timer latches the configured values into the
 *    active shadow register and enables the outputs at the same time.
 * 
 * The sequence can be aborted at any time by calling Motor::disarm().
 *
 */
void Motor::arm() {

  CRITICAL_SECTION() {
    // Reset controller states, integrators, setpoints, etc.
    // axis_->controller_.reset();
    armed_state_ = 1;
    is_armed_ = true;

  }
}

/**
 * @brief Updates the phase PWM timings unless the motor is disarmed.
 *
 * If the motor is armed, the PWM timings come into effect at the next update
 * event (and are enabled if they weren't already), unless the motor is disarmed
 * prior to that.
 * 
 * @param tentative: If true, the update is not counted as "refresh".
 */
void Motor::apply_pwm_timings(uint16_t timing, float torque_dir, bool tentative) {
    CRITICAL_SECTION() {
        TIM_HandleTypeDef* htim = timer_;
        TIM_TypeDef* tim = htim->Instance;
        switch (timer_channel_)
        {
        case 1:
          tim->CCR1 = timing;
          break;
        case 2:
          tim->CCR2 = timing;
          break;
        case 3:
          tim->CCR3 = timing;
          break;
        
        default:
          break;
        }

        HAL_GPIO_WritePin(M_DIR_GPIO_Port, M_DIR_Pin, (torque_dir > 0) ? GPIO_PIN_SET : GPIO_PIN_RESET);
        
        if (!tentative) {
            if (is_armed_) {
                // Set the Automatic Output Enable so that the Master Output Enable
                // bit will be automatically enabled on the next update event.
                tim->BDTR |= TIM_BDTR_AOE;
            }
        }
        
        // If a timer update event occurred just now while we were updating the
        // timings, we can't be sure what values the shadow registers now contain,
        // so we must disarm the motor.
        // (this also protects against the case where the update interrupt has too
        // low priority, but that should not happen)
        //if (__HAL_TIM_GET_FLAG(htim, TIM_FLAG_UPDATE)) {
        //    disarm_with_error(ERROR_CONTROL_DEADLINE_MISSED);
        //}
    }
}

float Motor::max_available_torque() {
    return config_.current_limit * config_.torque_constant;
}

/**
 * @brief Called when the underlying hardware timer triggers an update event.
 */
void Motor::dc_calib_cb(std::optional<float> current) {
    const float dc_calib_period = static_cast<float>(2 * TIM2_PERIOD_CLOCKS * TIM2_REPETITION) / APB1_TIM2_TIM14_FREQ;
    
        if (current.has_value()) {
            if (dc_calib_running_since_ <= config_.dc_calib_tau * 7.5f) {
                const float calib_filter_k = std::min(dc_calib_period / config_.dc_calib_tau, 1.0f);
                dc_calib_ += (current.value() - dc_calib_) * calib_filter_k;
                dc_calib_running_since_ += dc_calib_period;
            }
        } else {
            dc_calib_ = 0.0f;
            dc_calib_running_since_ = 0.0f;
        }
}


void Motor::update() {
    if (!is_armed_) {
        return;
    }
    std::optional<float> maybe_torque = torque_setpoint_src_.present();
    if (!maybe_torque.has_value()) {
        disarm_with_error(Error::ERROR_UNKNOWN_TORQUE);
        return;
    }
    std::optional<float> maybe_vel = vel_estimate_src_.present();
    if (!maybe_vel.has_value()) {
        disarm_with_error(Error::ERROR_UNKNOWN_VELOCITY);
        return;
    }
    float torque_abs = std::abs(maybe_torque.value());
    float torque_direction = (maybe_torque.value() > 0) ? config_.direction : -config_.direction;
    float work_direction = (maybe_torque.value() * maybe_vel.value() > 0) ? 1.0f : -1.0f;

    float current_setpoint = torque_abs / config_.torque_constant * work_direction;
    current_setpoint = std::clamp(current_setpoint, -config_.current_limit, config_.current_limit);

    float current_error = current_setpoint - current_meas_;
    float modulation = config_.current_p_gain * current_error + current_integral_;
    float modulation_abs = std::abs(modulation);
    if (modulation_abs > 1.0f) {
        modulation = std::copysign(1.0f, modulation);

        current_integral_ *= 0.9f; // anti-windup, this makes the integral term decay when the controller is saturated
    } else {
        current_integral_ += config_.current_i_gain * current_error * CURRENT_MEAS_PERIOD_S;
    }

    uint32_t timing = (uint32_t)((float)TIM2_PERIOD_CLOCKS * modulation_abs);
    apply_pwm_timings(timing, torque_direction, false);
}


