#ifndef __Z_MAIN_H
#define __Z_MAIN_H

#include "main.h"
#include "adc.h"
#include "dac.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "gpio.h"
#include "dma.h"

static constexpr float CURRENT_MEAS_PERIOD_S = 2 * ((float)TIM2_PERIOD_CLOCKS * (float)TIM2_REPETITION / (float)APB1_TIM2_TIM14_FREQ);

struct vel_command_t {
  float velocity;
  float torque;
  uint16_t timestamp;
};
struct tor_command_t {
  float torque;
  uint16_t timestamp;
};

#ifdef __cplusplus
extern "C" {

#include "controller.hpp"
#include "motor.hpp"
#include "stm32_system.h"
#include "stm32_spi_arbiter.hpp"
#include "stm32_gpio.hpp"
#include "encoder.hpp"

#include <optional>

}
#endif /* __cplusplus */

#endif // __Z_MAIN_H