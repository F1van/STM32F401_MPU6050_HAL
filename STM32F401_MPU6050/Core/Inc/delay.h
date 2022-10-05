#ifndef __DELAY_H
#define __DELAY_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "stdio.h"

/* Exported functions prototypes ---------------------------------------------*/

void tim_set_peak (TIM_TypeDef* TIMx, uint16_t period);   // Setting maximum value

void delay_ms(uint16_t delay);

/* Private defines -----------------------------------------------------------*/

#define F_AHB 72000000UL                // SysTick AHB clocking frequency

#define TIM_SET_UG(TIMx) TIMx->EGR |= TIM_EGR_UG      // Set update generation flag

#define TIM_RESET_UIF(TIMx) TIMx->SR &= ~TIM_SR_UIF  // Reset update interrupt flag

#define TRUE 1

#define FALSE 0

#define DELAY_OUTPUT_DEBUG 100000 // Set the value of the time to send debugging information

#define TIM_CLEAR_COUNTER(TIMx) TIMx->CNT = 0

/* Private variables ---------------------------------------------------------*/

static volatile _Bool check_update = FALSE;

static volatile _Bool check_data = FALSE;

static volatile uint32_t counter_delay_mpu = 0;

static volatile uint32_t counter_output_debug = 0;

static volatile uint32_t counter_reset_delay = 0;

static volatile double value_prev[5] = {0};

#endif /* __DELAY_H */
