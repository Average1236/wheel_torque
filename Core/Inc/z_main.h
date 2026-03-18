#ifndef __Z_MAIN_H
#define __Z_MAIN_H

#include "main.h"
#include "adc.h"
#include "dac.h"
// #include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "gpio.h"
#include "dma.h"

static constexpr float CURRENT_MEAS_PERIOD_S = 2 * ((float)TIM2_PERIOD_CLOCKS * 2) * (float)(TIM1_REPETITION + 1) / (float)APB2_TIM1_FREQ;

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

// Forward Declarations
class Motor;
class Controller;
// class Stm32SpiArbiter;
class Encoder;

#include "controller.hpp"
#include "motor.hpp"
#include "stm32_spi_arbiter.hpp"
#include "stm32_gpio.hpp"
#include "stm32_timer.hpp"
#include "encoder.hpp"

#include <optional>

enum Error {
    ERROR_NONE = 0,
    ERROR_CONTROLLER_UPDATE_FAILED = 1,
};

inline Error operator | (Error a, Error b) { return static_cast<Error>(static_cast<std::underlying_type_t<Error>>(a) | static_cast<std::underlying_type_t<Error>>(b)); }
inline Error operator & (Error a, Error b) { return static_cast<Error>(static_cast<std::underlying_type_t<Error>>(a) & static_cast<std::underlying_type_t<Error>>(b)); }
inline Error operator ^ (Error a, Error b) { return static_cast<Error>(static_cast<std::underlying_type_t<Error>>(a) ^ static_cast<std::underlying_type_t<Error>>(b)); }
inline Error& operator |= (Error &a, Error b) { return reinterpret_cast<Error&>(reinterpret_cast<std::underlying_type_t<Error>&>(a) |= static_cast<std::underlying_type_t<Error>>(b)); }
inline Error& operator &= (Error &a, Error b) { return reinterpret_cast<Error&>(reinterpret_cast<std::underlying_type_t<Error>&>(a) &= static_cast<std::underlying_type_t<Error>>(b)); }
inline Error& operator ^= (Error &a, Error b) { return reinterpret_cast<Error&>(reinterpret_cast<std::underlying_type_t<Error>&>(a) ^= static_cast<std::underlying_type_t<Error>>(b)); }
inline Error operator ~ (Error a) { return static_cast<Error>(~static_cast<std::underlying_type_t<Error>>(a)); }

#include "stm32_system.h"

#endif /* __cplusplus */

#endif // __Z_MAIN_H