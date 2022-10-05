#include "i2c.h"

void i2c_write_bits(I2C_HandleTypeDef *I2Cx, uint8_t dev_addr, uint8_t reg_addr, uint8_t length, uint8_t start_bit, uint8_t data)
{
    // Get data byte
    static uint8_t temp = 0;

    HAL_I2C_Mem_Read(I2Cx, dev_addr, reg_addr, 1, &temp, 1, i2c_timeout);

    temp &= ~(((1 << length) - 1) << start_bit);

    temp |= data << start_bit;

    HAL_I2C_Mem_Write(I2Cx, dev_addr, reg_addr, 1, &temp, 1, i2c_timeout);
}
