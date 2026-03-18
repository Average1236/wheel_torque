#ifndef __ENCODER_HPP
#define __ENCODER_HPP

#include "z_main.h"
#include "stm32_spi_arbiter.hpp"

class Encoder {
public:
    enum Error {
        ERROR_NONE                       = 0x00000000,
        ERROR_UNSTABLE_GAIN              = 0x00000001,
        ERROR_NO_RESPONSE                = 0x00000002,
        // ERROR_ABS_SPI_COM_FAIL           = 0x00000004,
    };
    struct Config_t {
        float bandwidth = 3000.0f; // PLL bandwidth [Hz]
        uint32_t cpr = 1 << 14; // counts per revolution
        float circular_range = 1.0f; // [turn] range of the circular tracking, usually set to 1 turn but can be set smaller for better low-speed performance at the cost of reduced max speed
        
        float direction = -1.0f; // 1 or -1, this is used to correct the direction of the encoder without having to change the wiring
    };

    Encoder(Stm32SpiArbiter* spi_arbiter, Stm32Gpio abs_spi_cs_gpio);

    void set_error(Error error);
    void update_pll_gains();
    void set_linear_count(int32_t count);
    void set_circular_count(int32_t count);

    void sample_now();
    bool update();

    Config_t config_;
    Error error_ = ERROR_NONE;

    Stm32SpiArbiter* spi_arbiter_;
    Stm32Gpio abs_spi_cs_gpio_;

    bool is_ready_ = false;
    int32_t shadow_count_ = 0;
    int32_t count_in_cpr_ = 0;
    float interpolation_ = 0.0f;
    OutputPort<float> phase_ = 0.0f;     // [rad]
    OutputPort<float> phase_vel_ = 0.0f; // [rad/s]
    float pos_estimate_counts_ = 0.0f;  // [count]
    float pos_cpr_counts_ = 0.0f;  // [count]
    float delta_pos_cpr_counts_ = 0.0f;  // [count] phase detector result for debug
    float vel_estimate_counts_ = 0.0f;  // [count/s]
    float pll_kp_ = 0.0f;   // [count/s / count]
    float pll_ki_ = 0.0f;   // [(count/s^2) / count]
    float calib_scan_response_ = 0.0f; // debug report from offset calib
    int32_t pos_abs_ = 0;
    float spi_error_rate_ = 0.0f; // fraction of failed SPI transactions

    OutputPort<float> pos_estimate_ = 0.0f; // [turn]
    OutputPort<float> vel_estimate_ = 0.0f; // [turn/s] raw PLL output
    OutputPort<float> vel_estimate_filtered_ = 0.0f; // [turn/s] low-pass filtered velocity
    OutputPort<float> pos_circular_ = 0.0f; // [turn]

    bool abs_spi_start_transaction();
    void abs_spi_cb(bool success);
    void abs_spi_cs_pin_init();
    static bool even_parity16(uint16_t value);
    static void as5047p_read_angle_cmd(uint16_t *data);
    bool abs_spi_pos_updated_ = false;
    bool as5047p_recovering_ = false;
    uint16_t abs_spi_dma_tx_[1] = {0xFFFF};
    uint16_t abs_spi_dma_rx_[1];
    Stm32SpiArbiter::SpiTask spi_task_;
};

// this is technically not thread-safe but practically it might be
inline Encoder::Error operator | (Encoder::Error a, Encoder::Error b) { return static_cast<Encoder::Error>(static_cast<std::underlying_type_t<Encoder::Error>>(a) | static_cast<std::underlying_type_t<Encoder::Error>>(b)); }
inline Encoder::Error operator & (Encoder::Error a, Encoder::Error b) { return static_cast<Encoder::Error>(static_cast<std::underlying_type_t<Encoder::Error>>(a) & static_cast<std::underlying_type_t<Encoder::Error>>(b)); }
inline Encoder::Error operator ^ (Encoder::Error a, Encoder::Error b) { return static_cast<Encoder::Error>(static_cast<std::underlying_type_t<Encoder::Error>>(a) ^ static_cast<std::underlying_type_t<Encoder::Error>>(b)); }
inline Encoder::Error& operator |= (Encoder::Error &a, Encoder::Error b) { return reinterpret_cast<Encoder::Error&>(reinterpret_cast<std::underlying_type_t<Encoder::Error>&>(a) |= static_cast<std::underlying_type_t<Encoder::Error>>(b)); }
inline Encoder::Error& operator &= (Encoder::Error &a, Encoder::Error b) { return reinterpret_cast<Encoder::Error&>(reinterpret_cast<std::underlying_type_t<Encoder::Error>&>(a) &= static_cast<std::underlying_type_t<Encoder::Error>>(b)); }
inline Encoder::Error& operator ^= (Encoder::Error &a, Encoder::Error b) { return reinterpret_cast<Encoder::Error&>(reinterpret_cast<std::underlying_type_t<Encoder::Error>&>(a) ^= static_cast<std::underlying_type_t<Encoder::Error>>(b)); }
inline Encoder::Error operator ~ (Encoder::Error a) { return static_cast<Encoder::Error>(~static_cast<std::underlying_type_t<Encoder::Error>>(a)); }


#endif // __ENCODER_HPP