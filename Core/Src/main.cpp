#include "z_main.h"

Motor motor(&htim2, TIM_CHANNEL_2);
Controller controller;

extern "C" {

int main(void)
{
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_ADC1_Init();
    MX_DAC_Init();
    MX_I2C1_Init();
    MX_SPI2_Init();
    MX_TIM2_Init();
    MX_TIM11_Init();
    MX_TIM14_Init();

    HAL_NVIC_SetPriority(TIM2_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(TIM2_IRQn);

    while (1);
}

} // extern "C"