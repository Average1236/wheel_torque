#include "encoder.hpp"

// debug variables
uint32_t pos_abs_debug = 0;
float pos_estimate_debug = 0;
float vel_estimate_debug = 0;
float pose_cirular_debug = 0;
uint16_t raw_val_debug = 0;
float spi_error_rate_debug = 0.0f;
float phase_debug = 0.0f;

static constexpr uint16_t AS5047P_REG_ANGLECOM = 0x3FFFU;
static constexpr uint16_t AS5047P_REG_ANGLEUNC = 0x3FFEU;
static constexpr uint16_t AS5047P_REG_ERRFL    = 0x0001U;
static constexpr uint16_t AS5047P_REG_PROG     = 0x0003U;
static constexpr uint16_t AS5047P_REG_DIAAGC   = 0x3FFCU;
static constexpr uint16_t AS5047P_CMD_READ_BIT = 0x4000U;
static constexpr uint16_t AS5047P_PARITY_BIT   = 0x8000U;
static constexpr uint16_t AS5047P_DATA_MASK    = 0x3FFFU;
static constexpr uint16_t AS5047P_EF_MASK      = 0x4000U;

Encoder::Encoder(Stm32SpiArbiter* spi_arbiter, Stm32Gpio abs_spi_cs_gpio) : spi_arbiter_(spi_arbiter), abs_spi_cs_gpio_(abs_spi_cs_gpio) {
    config_.cpr = 1 << 14; // default to 14 bit CPR since that's what the AS5047P has

    update_pll_gains();

    spi_task_.config = {
        .Mode = SPI_MODE_MASTER,
        .Direction = SPI_DIRECTION_2LINES,
        .DataSize = SPI_DATASIZE_16BIT,
        .CLKPolarity = SPI_POLARITY_LOW,
        .CLKPhase = SPI_PHASE_2EDGE,
        .NSS = SPI_NSS_SOFT,
        .BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64,
        .FirstBit = SPI_FIRSTBIT_MSB,
        .TIMode = SPI_TIMODE_DISABLE,
        .CRCCalculation = SPI_CRCCALCULATION_DISABLE,
        .CRCPolynomial = 10,
    };
    as5047p_read_angle_cmd(abs_spi_dma_tx_);
    abs_spi_cs_pin_init();
}

// two order PLL equations:
// \dot{\hat{\theta}} = \hat{\omega} + k_p ( \theta_{meas} - \hat{\theta} )
// \dot{\hat{\omega}} = k_i ( \theta_{meas} - \hat{\theta} )
void Encoder::update_pll_gains() {
    pll_kp_ = 2.0f * config_.bandwidth;  // basic conversion to discrete time
    pll_ki_ = 0.25f * (pll_kp_ * pll_kp_); // Critically damped

    // Check that we don't get problems with discrete time approximation
    if (!(CURRENT_MEAS_PERIOD_S * pll_kp_ < 1.0f)) {
        set_error(ERROR_UNSTABLE_GAIN);
    }
}

// Function that sets the current encoder count to a desired 32-bit value.
void Encoder::set_linear_count(int32_t count) {
    // Disable interrupts to make a critical section to avoid race condition
    uint32_t prim = cpu_enter_critical();

    // Update states
    shadow_count_ = count;
    pos_estimate_counts_ = (float)count;

    cpu_exit_critical(prim);
}

// Modulo (as opposed to remainder), per https://stackoverflow.com/a/19288271
inline int mod(const int dividend, const int divisor){
    int r = dividend % divisor;
    if (r < 0) r += divisor;
    return r;
}

// Function that sets the CPR circular tracking encoder count to a desired 32-bit value.
// Note that this will get mod'ed down to [0, cpr)
void Encoder::set_circular_count(int32_t count) {
    // Disable interrupts to make a critical section to avoid race condition
    uint32_t prim = cpu_enter_critical();

    // Update states
    count_in_cpr_ = mod(count, config_.cpr);
    pos_cpr_counts_ = (float)count_in_cpr_;

    cpu_exit_critical(prim);
}

void Encoder::sample_now() {
    abs_spi_start_transaction();
}

bool Encoder::even_parity16(uint16_t value) {
    bool parity = false;
    while (value != 0U) {
        parity = !parity;
        value &= (uint16_t)(value - 1U);
    }
    return parity;
}

void Encoder::as5047p_read_angle_cmd(uint16_t *data) {
    uint16_t cmd = (uint16_t)((AS5047P_REG_ANGLEUNC & AS5047P_DATA_MASK) | AS5047P_CMD_READ_BIT);
    // uint16_t cmd = (uint16_t)((AS5047P_REG_ERRFL & AS5047P_DATA_MASK) | AS5047P_CMD_READ_BIT);
    if (even_parity16(cmd)) {
        cmd |= AS5047P_PARITY_BIT;
    }
    data[0] = cmd;
}

bool Encoder::abs_spi_start_transaction() {
    if (Stm32SpiArbiter::acquire_task(&spi_task_)) {
        spi_task_.ncs_gpio = abs_spi_cs_gpio_;
        spi_task_.tx_buf = (uint8_t*)abs_spi_dma_tx_;
        spi_task_.rx_buf = (uint8_t*)abs_spi_dma_rx_;
        spi_task_.length = 1;
        spi_task_.on_complete = [](void* ctx, bool success) { ((Encoder*)ctx)->abs_spi_cb(success); };
        spi_task_.on_complete_ctx = this;
        spi_task_.next = nullptr;
        spi_arbiter_->transfer_async(&spi_task_);
    } else {
        return false;
    }
    return true;
}

void Encoder::abs_spi_cb(bool success) {
    uint16_t pos;

    if (success) {
        uint16_t frame = abs_spi_dma_rx_[0];

        raw_val_debug = frame;

        if (as5047p_recovering_) {
            as5047p_recovering_ = false;
            as5047p_read_angle_cmd(abs_spi_dma_tx_);
            goto done;
        }

        if (even_parity16(frame)) {
            goto done;
        }

        if ((frame & AS5047P_EF_MASK) != 0U) {
            // To clear the error flag, request the ERRFL register on the next transaction.
            uint16_t err_cmd = (uint16_t)((AS5047P_REG_ERRFL & AS5047P_DATA_MASK) | AS5047P_CMD_READ_BIT);
            if (even_parity16(err_cmd)) {
                err_cmd |= AS5047P_PARITY_BIT;
            }
            abs_spi_dma_tx_[0] = err_cmd;
            as5047p_recovering_ = true;
            goto done;
        }

        pos = frame & AS5047P_DATA_MASK;


        pos_abs_ = pos;
        abs_spi_pos_updated_ = true;
    }

done:
    Stm32SpiArbiter::release_task(&spi_task_);
}

void Encoder::abs_spi_cs_pin_init(){
    // Decode and init cs pin
    abs_spi_cs_gpio_.config(GPIO_MODE_OUTPUT_PP, GPIO_PULLUP);

    // Write pin high
    abs_spi_cs_gpio_.write(true);
}

// Wrap value to range.
// With default rounding mode (round to nearest),
// the result will be in range -y/2 to y/2
inline float wrap_pm(float x, float y) {
#ifdef FPU_FPV4
    float intval = (float)round_int(x / y);
#else
    float intval = nearbyintf(x / y);
#endif
    return x - intval * y;
}

// Same as fmodf but result is positive and y must be positive
inline float fmodf_pos(float x, float y) {
    float res = wrap_pm(x, y);
    if (res < 0) res += y;
    return res;
}

bool Encoder::update() {
    // update internal encoder state.
    int32_t delta_enc = 0;
    int32_t pos_abs_latched = pos_abs_; //LATCH


    if (abs_spi_pos_updated_ == false) {
        // Low pass filter the error
        spi_error_rate_ += CURRENT_MEAS_PERIOD_S * (1.0f - spi_error_rate_);
        // debug variable
        spi_error_rate_debug = spi_error_rate_;
        // debug
        if (spi_error_rate_debug > 0.95f) {
            volatile int a = 1;
        }
        // FIXME: Temporally disable error due to low reliability of MT6701 SPI
        // if (spi_error_rate_ > 0.05f) {
        //     set_error(ERROR_ABS_SPI_COM_FAIL);
        //     return false;
        // }
    } else {
        // Low pass filter the error
        spi_error_rate_ += CURRENT_MEAS_PERIOD_S * (0.0f - spi_error_rate_);
        // debug variable
        spi_error_rate_debug = spi_error_rate_;
    }

    // debug
    if (spi_error_rate_debug > 0.95f) {
        volatile int a = 1;
    }

    abs_spi_pos_updated_ = false;
    delta_enc = pos_abs_latched - count_in_cpr_; //LATCH
    delta_enc = mod(delta_enc, config_.cpr);
    if (delta_enc > config_.cpr/2) {
        delta_enc -= config_.cpr;
    }


    shadow_count_ += delta_enc;
    count_in_cpr_ += delta_enc;
    count_in_cpr_ = mod(count_in_cpr_, config_.cpr);

    count_in_cpr_ = pos_abs_latched;

    // Memory for pos_circular
    float pos_cpr_counts_last = pos_cpr_counts_;

    //// run pll (for now pll is in units of encoder counts)
    // Predict current pos
    pos_estimate_counts_ += CURRENT_MEAS_PERIOD_S * vel_estimate_counts_;
    pos_cpr_counts_      += CURRENT_MEAS_PERIOD_S * vel_estimate_counts_;
    // Encoder model
    auto encoder_model = [this](float internal_pos)->int32_t {
        return (int32_t)std::floor(internal_pos);
    };
    // discrete phase detector
    float delta_pos_counts = (float)(shadow_count_ - encoder_model(pos_estimate_counts_));
    float delta_pos_cpr_counts = (float)(count_in_cpr_ - encoder_model(pos_cpr_counts_));
    delta_pos_cpr_counts = wrap_pm(delta_pos_cpr_counts, (float)(config_.cpr));
    delta_pos_cpr_counts_ += 0.1f * (delta_pos_cpr_counts - delta_pos_cpr_counts_); // for debug
    // pll feedback
    pos_estimate_counts_ += CURRENT_MEAS_PERIOD_S * pll_kp_ * delta_pos_counts;
    pos_cpr_counts_ += CURRENT_MEAS_PERIOD_S * pll_kp_ * delta_pos_cpr_counts;
    pos_cpr_counts_ = fmodf_pos(pos_cpr_counts_, (float)(config_.cpr));
    vel_estimate_counts_ += CURRENT_MEAS_PERIOD_S * pll_ki_ * delta_pos_cpr_counts;
    bool snap_to_zero_vel = false;
    if (std::abs(vel_estimate_counts_) < 0.5f * CURRENT_MEAS_PERIOD_S * pll_ki_) {
        vel_estimate_counts_ = 0.0f;  //align delta-sigma on zero to prevent jitter
        snap_to_zero_vel = true;
    }

    // Outputs from Encoder for Controller
    pos_estimate_ = pos_estimate_counts_ / (float)config_.cpr;
    vel_estimate_ = vel_estimate_counts_ / (float)config_.cpr;

    // debug variables
    pos_abs_debug = pos_abs_;
    pos_estimate_debug = pos_estimate_counts_ / (float)config_.cpr;
    
    // TODO: we should strictly require that this value is from the previous iteration
    // to avoid spinout scenarios. However that requires a proper way to reset
    // the encoder from error states.
    float pos_circular = pos_circular_.any().value_or(0.0f);
    pos_circular +=  wrap_pm((pos_cpr_counts_ - pos_cpr_counts_last) / (float)config_.cpr, 1.0f);
    pos_circular = fmodf_pos(pos_circular, config_.circular_range);
    pos_circular_ = pos_circular;
    // debug variable
    pose_cirular_debug = pos_circular;

    return true;
}
