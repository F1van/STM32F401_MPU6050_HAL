#include "delay.h"

void tim_set_peak (TIM_TypeDef* TIMx, uint16_t period)
{
    TIMx->ARR = period - 1;
}

void delay_ms(uint16_t delay)
{
    counter_delay_mpu = 0;
    while(counter_delay_mpu == delay * 10);
}
