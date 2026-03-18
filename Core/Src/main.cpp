#include "z_main.h"

// Debug
volatile uint32_t debug_TIM1_UP_TIM10_IRQHandler_count = 0;
volatile uint32_t debug_ControlLoop_IRQHandler_count = 0;

#define ControlLoop_IRQHandler USART1_IRQHandler
#define ControlLoop_IRQn USART1_IRQn

Motor motor(&htim2, TIM_CHANNEL_2);
Controller controller;
Stm32SpiArbiter spi_arbiter{&hspi2};
Encoder encoder(&spi_arbiter, {GPIOB, GPIO_PIN_12}); // SPI2 CS on PB12

Error system_error = ERROR_NONE;
bool control_loop_running_ = false;
uint8_t i2c_rx_buf[3];
uint8_t i2c_tx_buf[3] = {0};

void start_timers() {
    CRITICAL_SECTION() {
        // Temporarily disable ADC triggers so they don't trigger as a side
        // effect of starting the timers.
        hadc1.Instance->CR2 &= ~ADC_CR2_EXTEN;

        /*
        * Synchronize TIM1, TIM8 and TIM13 such that:
        *  1. The triangle waveform of TIM1 leads the triangle waveform of TIM8 by a
        *     90° phase shift.
        *  2. Each TIM13 reload coincides with a TIM1 lower update event.
        */
        Stm32Timer::start_synchronously<2>(
            {&htim1, &htim2},
            {0, 0 /* TIM2 is on a clock as fast as TIM1 */}
        );

        hadc1.Instance->CR2 |= ADC_EXTERNALTRIGCONVEDGE_RISING;

        __HAL_ADC_CLEAR_FLAG(&hadc1, ADC_FLAG_EOC);
        __HAL_ADC_CLEAR_FLAG(&hadc1, ADC_FLAG_OVR);
        
        __HAL_TIM_CLEAR_IT(&htim1, TIM_IT_UPDATE);
        __HAL_TIM_ENABLE_IT(&htim1, TIM_IT_UPDATE);
    }
}

void start_adc_pwm() {
    motor.disarm();
    motor.timer_->Instance->CCR1 = 0;
    motor.timer_->Instance->CCR2 = 0;
    motor.timer_->Instance->CCR3 = 0;

    // Enable PWM outputs (they are still masked by MOE though)
    motor.timer_->Instance->CCER |= (TIM_CCx_ENABLE << motor.timer_channel_);
    motor.timer_->Instance->CCER |= (TIM_CCxN_ENABLE << motor.timer_channel_);

    // __HAL_ADC_ENABLE(&hadc1);
    HAL_ADC_Start(&hadc1);

    start_timers();

    __HAL_TIM_CLEAR_IT(&htim2, TIM_IT_UPDATE);
    __HAL_TIM_ENABLE_IT(&htim2, TIM_IT_UPDATE);
}

void start_control_loop() {
    CRITICAL_SECTION() {
        controller.vel_estimate_src_.connect_to(&encoder.vel_estimate_);
        controller.reset();
        controller.motor_ = &motor;
        motor.torque_setpoint_src_.connect_to(&controller.torque_out_);
    }

    if (!motor.is_armed_) {
        while (control_loop_running_); // wait until the control loop is not running to avoid racing with the control loop handler
        motor.arm();
    }
}

static bool fetch_and_reset_adc(std::optional<float>* current) {
    bool adc_done = (ADC1->SR & ADC_SR_JEOC) == ADC_SR_JEOC;
    if (!adc_done) {
        return false;
    }

    std::optional<float> current_value = motor.current_from_adc(ADC1->JDR1);
    if (current_value.has_value()) {
        *current = current_value;
    } else {
        *current = std::nullopt;
    }
    return true;
}

void control_loop_cb() {
    control_loop_running_ = true;
    // Reset all output ports so that we are certain about the freshness of
    // all values that we use.
    controller.torque_out_.reset();
    encoder.vel_estimate_.reset();

    encoder.update();
    if (!controller.update()) {
        system_error |= ERROR_CONTROLLER_UPDATE_FAILED;
    }
    motor.update();
    control_loop_running_ = false;
}

extern "C" {

void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi) {
    HAL_SPI_TxRxCpltCallback(hspi);
}

void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi) {
    HAL_SPI_TxRxCpltCallback(hspi);
}

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi) {
    if (hspi == &hspi2) {
        spi_arbiter.on_complete();
    }
}

volatile uint32_t repetition = 0;
volatile uint32_t timestamp_ = 0;
volatile bool counting_down_ = false;

void TIM1_UP_TIM10_IRQHandler(void) {
    __HAL_TIM_CLEAR_FLAG(&htim1, TIM_FLAG_UPDATE);

    debug_TIM1_UP_TIM10_IRQHandler_count++;

    // hadc1.Instance->CR2 |= ADC_CR2_SWSTART; // trigger ADC conversion, we will fetch the result in the control loop handler

    volatile bool counting_down = TIM1->CR1 & TIM_CR1_DIR;
    bool timer_update_missed = (counting_down == counting_down_);
    if (timer_update_missed) {
        motor.disarm_with_error(Motor::Error::ERROR_TIMER_UPDATE_MISSED);
    }

    counting_down_ = counting_down;
    timestamp_ += TIM2_PERIOD_CLOCKS * (TIM1_REPETITION + 1);

    if (!counting_down) {
    // if (counting_down) {
        encoder.sample_now();
        NVIC->STIR = ControlLoop_IRQn;
    } else {
        // Tentatively reset the PWM output to 0. If the control
        // loop handler finishes in time then these values will be overridden
        // before they go into effect.
        // motor.apply_pwm_timings(0, 1.0f, true);
    }
}

void ControlLoop_IRQHandler(void) {
    debug_ControlLoop_IRQHandler_count++;
    uint32_t timestamp = timestamp_;

    std::optional<float> current;
    // Ensure that ADC is done
    while (!(ADC1->SR & ADC_SR_JEOC));
    if (!fetch_and_reset_adc(&current)) {
        motor.disarm_with_error(Motor::Error::ERROR_BAD_TIMING);
    }
    motor.current_meas_cb(current);

    control_loop_cb();

    // By this time the ADCs for both M0 and M1 should have fired again. But
    // let's wait for them just to be sure.
    while (!(ADC1->SR & ADC_SR_JEOC));
    while (!counting_down_);
    // while (counting_down_);

    if (!fetch_and_reset_adc(&current)) {
        motor.disarm_with_error(Motor::Error::ERROR_BAD_TIMING);
    }
    // Not running after dc_calib_valid
    motor.dc_calib_cb(current);
    // motor.current_meas_cb(current);

    motor.pwm_update_cb();

    volatile uint32_t a = 0;

    if (timestamp_ != timestamp + TIM2_PERIOD_CLOCKS * (TIM1_REPETITION + 1)) {
        motor.disarm_with_error(Motor::Error::ERROR_CONTROL_DEADLINE_MISSED);
    }

}

int main(void)
{
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_ADC1_Init();
    MX_DAC_Init();
    // MX_I2C1_Init();
    MX_SPI2_Init();
    MX_TIM1_Init();
    MX_TIM2_Init();
    MX_TIM11_Init();
    MX_TIM14_Init();

    HAL_NVIC_SetPriority(ControlLoop_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(ControlLoop_IRQn);

    HAL_NVIC_SetPriority(TIM1_UP_TIM10_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(TIM1_UP_TIM10_IRQn);

    // HAL_I2C_Slave_Receive_DMA(&hi2c1, i2c_rx_buf, sizeof(i2c_rx_buf));

    HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET);

    HAL_DAC_SetValue(&hdac, DAC_CHANNEL_2, DAC_ALIGN_12B_R, 4095);
    HAL_DAC_Start(&hdac, DAC_CHANNEL_2);

    HAL_GPIO_WritePin(M_MODE_GPIO_Port, M_MODE_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(M_RESET_GPIO_Port, M_RESET_Pin, GPIO_PIN_SET);

    start_adc_pwm();

    HAL_Delay(10);

    for (uint32_t i = 0; i < 10000; ++i) {
        if (motor.current_meas_.has_value() && motor.dc_calib_valid_) {
            break;
        }
        HAL_Delay(1);
    }

    start_control_loop();

    // HAL_Delay(1000);

    // controller.config_.set_control_mode(Controller::MODE_TORQUE_CONTROL);
    // controller.input_torque_ = -0.005f;

    // // controller.config_.set_control_mode(Controller::MODE_VELOCITY_CONTROL);
    // // controller.input_vel_ = 10.0f;

    // HAL_Delay(5000);

    // controller.input_torque_ = 0.0f;
    // controller.input_vel_ = 0.0f;

    while (1);
}

} // extern "C"