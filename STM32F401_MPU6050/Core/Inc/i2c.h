/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __I2C_H
#define __I2C_H

/* Private includes ----------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "main.h"

/* Exported constants --------------------------------------------------------*/
static volatile const uint16_t i2c_timeout = 100;

/* Exported functions prototypes ---------------------------------------------*/
void i2c_write_bits(I2C_HandleTypeDef *I2Cx, uint8_t dev_addr, uint8_t reg_addr, uint8_t length, uint8_t start_bit, uint8_t data);



#endif /* __I2C_H */
