#include "mpu6050.h"

uint8_t Data;

float a_norm;

float g_norm;

//=============================================================================
// Read address MPU from WHO_AM_I
uint8_t read_MPU_ID(I2C_HandleTypeDef *I2Cx)
{
    uint8_t status = 1;
    uint8_t MPU_ID = 0;

    HAL_I2C_Mem_Read(I2Cx, MPU6050_ADDR, WHO_AM_I, 1, &MPU_ID, 1, i2c_timeout);

    if (MPU_ID == 0x68) printf("MPU6050.......OK\r\n");
    else if (MPU_ID == 0x71) {
        printf("MPU9250.......OK\r\n");
        status = 0;
    } else {
        printf("MPU.........%x\r\n", MPU_ID);
        status = 1;
    }
    return status;
}

//=============================================================================
// Reset MPU6050, coming out of sleep mode, synchronization source: PLL with X Gyro reference
void MPU6050_reset_wakeup(I2C_HandleTypeDef *I2Cx)
{
    // reset MPU6050
    Data |= 1 << DEVICE_RESET;
    HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, PWR_MGMT_1, 1, &Data, 1, i2c_timeout);

    // Delay 100 ms for PLL to get established on x-axis gyro;
    HAL_Delay(150);

    // Set synchronization source: CLKSEL = 3 - PLL with Z Gyro reference
    Data = 0;
    Data = 0x28;
    HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, PWR_MGMT_1, 1, &Data, 1, i2c_timeout);

    // Delay 100 ms for PLL to get established on x-axis gyro;
    HAL_Delay(150);
}

//=============================================================================
// Initializing MPU6050
void MPU6050_init(I2C_HandleTypeDef *I2Cx)
{
    // Set sample rate
    MPU6050_set_sample_rate(I2Cx);

    // setting the filter bandwidth to 42 Hz
    i2c_write_bits(I2Cx, MPU6050_ADDR, CONFIG, 3, DLPF_CFG, DLPF);

    MPU6050_set_accel_scale(I2Cx);

    MPU6050_set_gyro_scale(I2Cx);

    // scaling factors calculation
    MPU6050_get_norm(&a_norm, &g_norm);

    Data = 0;
    Data = 0x40;
    HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, PWR_MGMT_2, 1, &Data, 1, i2c_timeout);

    // interrupt output setting
    // the logic level for the INT pin is active high;
    // the INT pin is configured as open drain;
    // the INT pin is held high until the interrupt is cleared;
    // interrupt status bits are cleared only by reading INT_STATUS
    // interrupt reset only by reading INT_STATUS

    // interrupts on data availability
    Data = 0;
    Data |= 1 << DATA_RDY_EN;
    HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, INT_ENABLE, 1, &Data, 1, i2c_timeout);

    HAL_Delay(5);

    printf("MPU init......OK\r\n");
}

void MPU6050_Read_Accel(I2C_HandleTypeDef *I2Cx, MPU6050_t *DataStruct)
{
    uint8_t Rec_Data[6];

    // Read 6 BYTES of data starting from ACCEL_XOUT_H register

    HAL_I2C_Mem_Read(I2Cx, MPU6050_ADDR, ACCEL_XOUT_H, 1, Rec_Data, 6, i2c_timeout);

    DataStruct->Accel_X_RAW = (int16_t)(Rec_Data[0] << 8 | Rec_Data[1]);

    DataStruct->Accel_Y_RAW = (int16_t)(Rec_Data[2] << 8 | Rec_Data[3]);

    DataStruct->Accel_Z_RAW = (int16_t)(Rec_Data[4] << 8 | Rec_Data[5]);
}

void MPU6050_Read_Gyro(I2C_HandleTypeDef *I2Cx, MPU6050_t *DataStruct)
{
    uint8_t Rec_Data[6];

    // Read 6 BYTES of data starting from GYRO_XOUT_H register

    HAL_I2C_Mem_Read(I2Cx, MPU6050_ADDR, GYRO_XOUT_H, 1, Rec_Data, 6, i2c_timeout);

    DataStruct->Gyro_X_RAW = (int16_t)(Rec_Data[0] << 8 | Rec_Data[1]);

    DataStruct->Gyro_Y_RAW = (int16_t)(Rec_Data[2] << 8 | Rec_Data[3]);

    DataStruct->Gyro_Z_RAW = (int16_t)(Rec_Data[4] << 8 | Rec_Data[5]);
}

void MPU6050_Read_Temp(I2C_HandleTypeDef *I2Cx, MPU6050_t *DataStruct)
{
    uint8_t Rec_Data[2];

    int16_t temp;

    // Read 2 BYTES of data starting from TEMP_OUT_H_REG register

    HAL_I2C_Mem_Read(I2Cx, MPU6050_ADDR, TEMP_OUT_H, 1, Rec_Data, 2, i2c_timeout);

    temp = (int16_t)(Rec_Data[0] << 8 | Rec_Data[1]);

    DataStruct->Temperature = (float)((int16_t)temp / (float)340.0 + (float)36.53);
}

void MPU6050_Read_All(I2C_HandleTypeDef *I2Cx, MPU6050_t *DataStruct)
{
    // Read 14 BYTES of data starting from ACCEL_XOUT_H register

    MPU6050_Read_Accel(I2Cx, DataStruct);

    MPU6050_Read_Gyro(I2Cx, DataStruct);

    MPU6050_Read_Temp(I2Cx, DataStruct);
}

double smooth(double *input, double window)
{
    int i, j, z, k1, k2, hw;
    double tmp;
    double output;
    if(fmod(window, 2) == 0) window++;
    hw = (window - 1) / 2;
    output = input[0];

    for (i = 1; i < window; i++) {
        tmp = 0;
        if(i < hw) {
            k1 = 0;
            k2 = 2 * i;
            z = k2 + 1;
        } else if((i + hw) > (window - 1)) {
            k1 = i - window + i + 1;
            k2 = window - 1;
            z = k2 - k1 + 1;
        } else {
            k1 = i - hw;
            k2 = i + hw;
            z = window;
        }
    }

    for (j = k1; j <= k2; j++) {
        tmp = tmp + input[j];
    }
    return output = tmp / z;
}
//=============================================================================
// Adjusting the accelerometer sensitivity
void MPU6050_set_accel_scale(I2C_HandleTypeDef * I2Cx)
{
    switch (ACCEL_FS) {
    case 2:
        Data = 0;
        HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, ACCEL_CONFIG, 1, &Data, 1, i2c_timeout);
        break;
    case 4:
        Data |= 1 << AFS_SEL;
        HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, ACCEL_CONFIG, 1, &Data, 1, i2c_timeout);
        break;
    case 8:
        Data |= 2 << AFS_SEL;
        HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, ACCEL_CONFIG, 1, &Data, 1, i2c_timeout);
        break;
    case 16:
        Data |= 3 << AFS_SEL;
        HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, ACCEL_CONFIG, 1, &Data, 1, i2c_timeout);
        break;
    }
}

//=============================================================================
// Adjusting the gyroscope sensitivity
void MPU6050_set_gyro_scale(I2C_HandleTypeDef * I2Cx)
{
    switch (GYRO_FS) {
    case 250:
        Data = 0;
        HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, GYRO_CONFIG, 1, &Data, 1, i2c_timeout);
        break;
    case 500:
        Data |= 1 << GFS_SEL;
        HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, GYRO_CONFIG, 1, &Data, 1, i2c_timeout);
        break;
    case 1000:
        Data |= 2 << GFS_SEL;
        HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, GYRO_CONFIG, 1, &Data, 1, i2c_timeout);
        break;
    case 2000:
        Data |= 3 << GFS_SEL;
        HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, ACCEL_CONFIG, 1, &Data, 1, i2c_timeout);
        break;
    }
}

//=============================================================================
// Setting the sampling rate
void MPU6050_set_sample_rate(I2C_HandleTypeDef * I2Cx)
{
    uint8_t sample_div;
    switch(DLPF) {
    case 0:
        if(SAMPLE_RATE > 8000) sample_div = 0;
        else if(SAMPLE_RATE < 32) sample_div = 249;
        else sample_div = 8000u / SAMPLE_RATE - 1u;
        break;
    case 7:
        if(SAMPLE_RATE > 8000) sample_div = 0;
        else if(SAMPLE_RATE < 32) sample_div = 249;
        else sample_div = 8000u / SAMPLE_RATE - 1u;
        break;
    default:
        if(SAMPLE_RATE > 1000) sample_div = 0;
        else if(SAMPLE_RATE < 4) sample_div = 249;
        else sample_div = 1000u / SAMPLE_RATE - 1u;
    }
    HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, SMPLRT_DIV, 1, &sample_div, 1, i2c_timeout);
}

void MPU6050_get_norm(float * p_a_norm, float * p_g_norm)
{
    switch (ACCEL_FS) {
    // ACCEL_FS: ±2g; ±4g; ±8g; ±16g
    case 2:
        *p_a_norm = 2.f / 32768.f;
        break;
    case 4:
        *p_a_norm = 4.f / 32768.f;
        break;
    case 8:
        *p_a_norm = 8.f / 32768.f;
        break;
    case 16:
        *p_a_norm = 16.f / 32768.f;
        break;
    }
    switch (GYRO_FS) {
    // GYRO_FS: ±250°/сек; ±500°/сек; ±1000°/сек; ±2000°/сек
    case 250:
        *p_g_norm = 250.f / 32768.f;
        break;
    case 500:
        *p_g_norm = 500.f / 32768.f;
        break;
    case 1000:
        *p_g_norm = 1000.f / 32768.f;
        break;
    case 2000:
        *p_g_norm = 2000.f / 32768.f;
        break;
    }
}

//=============================================================================
// Calibrate MPU6050
void MPU_calibrate(I2C_HandleTypeDef * I2Cx)
{
    Data = 0;
    Data |= 1 << DLPF_CFG;
    HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, CONFIG, 1, &Data, 1, i2c_timeout);

    HAL_Delay(5);
    printf("MPU calibr....OK\r\n");
}

//=============================================================================
// should return percent deviation from factory trim values,
// +/- 14 or less deviation is a pass
void MPU6050_self_test(I2C_HandleTypeDef * I2Cx)
{
    uint8_t data[12];
    int32_t gAvg[3] = {0}, aAvg[3] = {0}, aSTAvg[3] = {0}, gSTAvg[3] = {0};
    uint8_t Xa_test, Ya_test, Za_test, Xg_test, Yg_test, Zg_test;
    float FT_Xa, FT_Ya, FT_Za, FT_Xg, FT_Yg, FT_Zg;
    float percent_Xa, percent_Ya, percent_Za, percent_Xg, percent_Yg, percent_Zg;

    MPU6050_reset_wakeup(I2Cx);

    Data = 0;
    Data |= 2 << DLPF_CFG;
    HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, CONFIG, 1, &Data, 1, i2c_timeout);
    // configure the accelerometer for self-test
    // When performing accelerometer self test, the full-scale range should be set to ±8g
    Data = 0;
    Data |= 2 << AFS_SEL;
    HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, ACCEL_CONFIG, 1, &Data, 1, i2c_timeout);
    // configure the gyroscope for self-test
    // When performing self test for the gyroscope, the full-scale range should be set to ±250dps.
    Data = 0;
    HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, GYRO_CONFIG, 1, &Data, 1, i2c_timeout);
    // delay a while to let the device stabilyze
    HAL_Delay(25);

    // get average values of gyro and accelerometer output without self-test
    // (1kHz rate, 200 readings)
    for(uint8_t i = 0; i < 200; i++) {
        Data = 0;
        HAL_I2C_Mem_Read(I2Cx, MPU6050_ADDR, INT_STATUS, 1, &Data, 1, i2c_timeout);
        if((Data & 0x01) == 0);
        HAL_I2C_Mem_Read(I2Cx, MPU6050_ADDR, ACCEL_XOUT_H, 1, data, 6, i2c_timeout);
        HAL_I2C_Mem_Read(I2Cx, MPU6050_ADDR, GYRO_XOUT_H, 1, data + 6, 6, i2c_timeout);
        aAvg[0] += (int16_t)(data[0] << 8 | data[1]);
        aAvg[1] += (int16_t)(data[2] << 8 | data[3]);
        aAvg[2] += (int16_t)(data[4] << 8 | data[5]);
        gAvg[0] += (int16_t)(data[6] << 8 | data[7]);
        gAvg[1] += (int16_t)(data[8] << 8 | data[9]);
        gAvg[2] += (int16_t)(data[10] << 8 | data[11]);
    }
    for (uint8_t i = 0; i < 2; i++) {
        // Get average of 200 values and store as average current readings
        aAvg[i] /= 200;
        gAvg[i] /= 200;
    }

    // configure the accelerometer for self-test
    // When performing accelerometer self test, the full-scale range should be set to ±8g
    Data = 0;
    Data |= 1 << XA_ST | 1 << YA_ST | 1 << ZA_ST | 2 << AFS_SEL;
    HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, ACCEL_CONFIG, 1, &Data, 1, i2c_timeout);
    // configure the gyroscope for self-test
    // When performing self test for the gyroscope, the full-scale range should be set to ±250dps.
    Data = 0;
    Data |= 1 << XG_ST | 1 << YG_ST | 1 << ZG_ST;
    HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, GYRO_CONFIG, 1, &Data, 1, i2c_timeout);
    // delay a while to let the device stabilyze
    HAL_Delay(25);

    // get average values of gyro and accelerometer self-test output
    // (1kHz rate, 200 readings)
    for(uint8_t i = 0; i < 200; i++) {
        Data = 0;
        HAL_I2C_Mem_Read(I2Cx, MPU6050_ADDR, INT_STATUS, 1, &Data, 1, i2c_timeout);
        if((Data & 0x01) == 0);
        HAL_I2C_Mem_Read(I2Cx, MPU6050_ADDR, ACCEL_XOUT_H, 1, data, 6, i2c_timeout);
        HAL_I2C_Mem_Read(I2Cx, MPU6050_ADDR, GYRO_XOUT_H, 1, data + 6, 6, i2c_timeout);
        aSTAvg[0] += (int16_t)(data[0] << 8 | data[1]);
        aSTAvg[1] += (int16_t)(data[2] << 8 | data[3]);
        aSTAvg[2] += (int16_t)(data[4] << 8 | data[5]);
        gSTAvg[0] += (int16_t)(data[6] << 8 | data[7]);
        gSTAvg[1] += (int16_t)(data[8] << 8 | data[9]);
        gSTAvg[2] += (int16_t)(data[10] << 8 | data[11]);
    }
    for (uint8_t i = 0; i < 2; i++) {
        // Get average of 200 values and store as average current readings
        aSTAvg[i] /= 200L;
        gSTAvg[i] /= 200L;
    }

    // data[0] - X-axis self-test results
    // data[1] - Y-axis self-test results
    // data[2] - Z-axis self-test results
    // data[3] - mixed-axis self-test results
    HAL_I2C_Mem_Read(I2Cx, MPU6050_ADDR, SELF_TEST_X, 1, data, 4, i2c_timeout);

    // extract the acceleration test results first
    Xa_test = (data[0] & 0xE0) >> 3 | (data[3] & 0x30) >> 4;				// XA_TEST result is a five-bit unsigned integer
    Ya_test = (data[1] & 0xE0) >> 3 | (data[3] & 0x0C) >> 2;				// YA_TEST result is a five-bit unsigned integer
    Za_test = (data[2] & 0xE0) >> 3 | (data[3] & 0x03) >> 0;				// ZA_TEST result is a five-bit unsigned integer

    // extract the gyration test results first
    Xg_test = data[0] & 0x1F;										// XG_TEST result is a five-bit unsigned integer
    Yg_test = data[1] & 0x1F;										// YG_TEST result is a five-bit unsigned integer
    Zg_test = data[2] & 0x1F;										// ZG_TEST result is a five-bit unsigned integer

    // obtaining the accelerometer and gyroscope Factory Trim (FT) value
    FT_Xa = 4096.f * 0.34f * powf((0.92f / 0.34f), ((Xa_test - 1.f) / 30.f));	// FT[Xa] factory trim calculation
    FT_Ya = 4096.f * 0.34f * powf((0.92f / 0.34f), ((Ya_test - 1.f) / 30.f));	// FT[Ya] factory trim calculation
    FT_Za = 4096.f * 0.34f * powf((0.92f / 0.34f), ((Za_test - 1.f) / 30.f));	// FT[Za] factory trim calculation
    FT_Xg =  25.0f * 131.f * powf(1.046f, (Xg_test - 1.f));						// FT[Xg] factory trim calculation
    FT_Yg = -25.0f * 131.f * powf(1.046f, (Yg_test - 1.f));						// FT[Yg] factory trim calculation
    FT_Zg =  25.0f * 131.f * powf(1.046f, (Zg_test - 1.f));						// FT[Zg] factory trim calculation

    // report results as a ratio of (STR-FT)/FT;
    // the change from Factory Trim of the Self-Test Response
    // to get to percent, must multiply by 100 and subtract result from 100
    percent_Xa = 100.f * (aSTAvg[0] - aAvg[0] - FT_Xa) / FT_Xa;
    percent_Ya = 100.f * (aSTAvg[1] - aAvg[1] - FT_Ya) / FT_Ya;
    percent_Za = 100.f * (aSTAvg[2] - aAvg[2] - FT_Za) / FT_Za;
    percent_Xg = 100.f * (gSTAvg[0] - gAvg[0] - FT_Xg) / FT_Xg;
    percent_Yg = 100.f * (gSTAvg[1] - gAvg[1] - FT_Yg) / FT_Yg;
    percent_Zg = 100.f * (gSTAvg[2] - gAvg[2] - FT_Zg) / FT_Zg;

    HAL_Delay(5);

    printf("%4.2f %4.2f %4.2f\r\n", percent_Xa, percent_Ya, percent_Za);

    HAL_Delay(5);

    printf("%4.2f %4.2f %4.2f\r\n", percent_Xg, percent_Yg, percent_Zg);
}
