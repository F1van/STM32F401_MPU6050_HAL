#ifndef __MPU6050_H
#define __MPU6050_H

#include <stdint.h>
#include "stm32f4xx_hal.h"
#include <math.h>
#include "delay.h"
#include "i2c.h"

#define Kf 0.05f

#define WHO_AM_I			0x75

// Setup MPU6050
#define MPU6050_ADDR 0xD0

#define INT_STATUS 0x3A

#define  DEG   180.f/M_PI

#define  RAD   M_PI/180.f

#define SMPLRT_DIV     0x19
#define CONFIG         0x1A
// CONFIG bits:
#define EXT_SYNC_SET		3
#define DLPF_CFG			0
#define DLPF				1			 // low-pass filter

// EXT_SYNC_SET:
#define INPUT_DISABLED		0
#define TEMP_OUT_L0			1
#define GYRO_XOUT_L0		2
#define GYRO_YOUT_L0		3
#define GYRO_ZOUT_L0		4
#define ACCEL_XOUT_L0		5
#define ACCEL_YOUT_L0		6
#define ACCEL_ZOUT_L0		7

#define ACCEL_CONFIG   0x1C
#define ACCEL_FS	   2				// ACCEL_FS: ±2g; ±4g; ±8g;

// ACCEL_CONFIG bits:
#define XA_ST				7
#define YA_ST				6
#define ZA_ST				5
#define AFS_SEL				3
//=======================================================================
#define ACCEL_XOUT_H        0x3B
#define TEMP_OUT_H          0x41

#define GYRO_CONFIG         0x1B
#define GYRO_FS		        2000

// GYRO_CONFIG bits
#define XG_ST				7
#define YG_ST				6
#define ZG_ST			    5
#define GFS_SEL				3
#define GYRO_XOUT_H    0x43

//=======================================================================
#define SAMPLE_RATE		    200					// Sampling rate, Hz
#define DLPF			    1

#define INT_PIN_CFG			0x37
// INT_PIN_CFG bits:
#define INT_LEVEL			7
#define INT_OPEN			6
#define LATCH_INT_EN		5
#define INT_RD_CLEAR		4
#define FSYNC_INT_LEVEL		3
#define FSYNC_INT_EN		2
#define I2C_BYPASS_EN		1
#define INTCFG_CLKOUT_EN	0

#define LSB_ACCEL_SENSITIVITY 16384.0
#define LSB_GYRO_SENSITIVITY 131.0

//=======================================================================
// * - is not mentioned in manual
#define XG_OFFS_TC			0x00					// *[7] PWR_MODE, [6:1] XG_OFFS_TC, [0] OTP_BNK_VLD

#define YG_OFFS_TC			0x01					// *[7] PWR_MODE, [6:1] YG_OFFS_TC, [0] OTP_BNK_VLD
#define ZG_OFFS_TC			0x02					// *[7] PWR_MODE, [6:1] ZG_OFFS_TC, [0] OTP_BNK_VLD
// _G_OFFS_TC bits:
#define PWR_MODE			7
#define XG_OFFS_TC_0		1
#define YG_OFFS_TC_0		1
#define ZG_OFFS_TC_0		1
#define OTP_BNK_VLD			0

//===========================

#define X_FINE_GAIN			0x03					// *[7:0] X_FINE_GAIN
#define Y_FINE_GAIN			0x04					// *[7:0] Y_FINE_GAIN
#define Z_FINE_GAIN			0x05					// *[7:0] Z_FINE_GAIN

// User-defined trim values for accelerometer
// ±16g Offset cancellation in all Full Scale modes,
// 15 bit 0,98-mg per steps
#define XA_OFFSET_H			0x06					// *[7:0] bit 14:7 XA_OFFS_USR
#define XA_OFFSET_L			0x07					// *[7:1] bit  6:0 XA_OFFS_USR; [0] - reserved
#define YA_OFFSET_H			0x08					// *[7:0] bit 14:7 YA_OFFS_USR
#define YA_OFFSET_L			0x09					// *[7:1] bit  6:0 YA_OFFS_USR; [0] - reserved
#define ZA_OFFSET_H			0x0A					// *[7:0] bit 14:7 ZA_OFFS_USR
#define ZA_OFFSET_L			0x0B					// *[7:1] bit  6:0 ZA_OFFS_USR; [0] - reserved
#define MPU_PROD_ID_REG		0X0C					// *PROD_ID register

#define SELF_TEST_X			0x0D
#define SELF_TEST_Y			0x0E
#define SELF_TEST_Z			0x0F
#define SELF_TEST_A			0x10

#define FIFO_EN				0x23

// FIFO_EN bits:
#define TEMP_FIFO_EN		7
#define XG_FIFO_EN			6
#define YG_FIFO_EN			5
#define ZG_FIFO_EN			4
#define ACCEL_FIFO_EN		3
#define SLV2_FIFO_EN		2
#define SLV1_FIFO_EN		1
#define SLV0_FIFO_EN		0

//=======================================================================
#define FIFO_COUNT_H		0x72
#define FIFO_COUNT_L		0x73
#define FIFO_R_W			0x74

#define MPU6050_FIFO_DEFAULT_TIMEOUT	11000

#define PWR_MGMT_1			0x6B
#define PWR_MGMT_2          0x6C

/*
 *
 * CLK_SEL | Clock Source
 * --------+--------------------------------------
 * 0       | Internal oscillator 8 MHz
 * 1       | PLL with X Gyro reference
 * 2       | PLL with Y Gyro reference
 * 3       | PLL with Z Gyro reference
 * 4       | PLL with external 32.768kHz reference
 * 5       | PLL with external 19.2MHz reference
 * 6       | Reserved
 * 7       | Stops the clock and keeps the timing generator in reset
 *
 */
// PWR_MGMT_1 bits:
#define DEVICE_RESET		7						// Resets all internal registers to their default values
#define SLEEP				6						// Sleep mode
#define CYCLE				5						// Cyclic alternation of standby and wake-up modes
#define TEMP_DIS			3						// Disables the temperature sensor
#define CLKSEL				0						// Device clock source

//=======================================================================

#define FF_THR				0x1D					// *Free-fall
#define FF_DUR				0x1E					// *Free-fall
#define MOT_THR				0x1F					// *Motion detection threshold bits [7:0]
// Wake-on Motion Threshold - this register holds the threshold value for the
// Wake on Motion Interrupt for accel x/y/z axes. LSB = 4mg. Range is 0mg to 1020mg.
#define MOT_DUR				0x20					// *Duration counter threshold for motion interrupt generation, 1 kHz rate, LSB = 1 ms
#define ZMOT_THR			0x21					// *Zero-motion detection threshold bits [7:0]
#define ZRMOT_DUR			0x22					// *Duration counter threshold for zero motion interrupt generation, 16 Hz rate, LSB = 64 ms

#define INT_ENABLE          0x38
// INT_ENABLE bits:
#define FF_EN				7
#define MOT_EN				6
#define ZMOT_EN				5
#define FIFO_OFLOW_EN		4
#define I2C_MST_INT_EN		3
#define PLL_RDY_INT_EN		2
#define DMP_INT_EN          1
#define DATA_RDY_EN         0

#define MOT_DETECT_CTRL		0x69
#define USER_CTRL			0x6A

// USER_CTRL bits:
#define DMP_EN_BIT			7
#define FIFO_EN_BIT			6
#define I2C_MST_EN			5
#define I2C_IF_DIS			4
#define DMP_RESET			3
#define FIFO_RESET			2
#define I2C_MST_RESET		1
#define SIG_COND_RESET		0

//=======================================================================

#define I2C_MST_CTRL		0x24
#define I2C_SLV0_ADDR		0x25
#define I2C_SLV0_REG		0x26
#define I2C_SLV0_CTRL		0x27
#define I2C_SLV1_ADDR		0x28
#define I2C_SLV1_REG		0x29
#define I2C_SLV1_CTRL		0x2A
#define I2C_SLV2_ADDR		0x2B
#define I2C_SLV2_REG		0x2C
#define I2C_SLV2_CTRL		0x2D
#define I2C_SLV3_ADDR		0x2E
#define I2C_SLV3_REG		0x2F
#define I2C_SLV3_CTRL		0x30
#define I2C_SLV4_ADDR		0x31
#define I2C_SLV4_REG		0x32
#define I2C_SLV4_DO			0x33
#define I2C_SLV4_CTRL		0x34
#define I2C_SLV4_DI			0x35
#define I2C_MST_STATUS		0x36

static volatile const double Accel_Z_corrector = 14418.0;
static volatile uint32_t timer;

//=======================================================================
// MPU6050 structure
typedef struct {
    int16_t Accel_X_RAW;
    int16_t Accel_Y_RAW;
    int16_t Accel_Z_RAW;

    int16_t Gyro_X_RAW;
    int16_t Gyro_Y_RAW;
    int16_t Gyro_Z_RAW;

    float Temperature;

    double smooth_Ax;
    double smooth_Ay;
    double smooth_Az;

    double smooth_Gx;
    double smooth_Gy;
    double smooth_Gz;
} MPU6050_t;

//=======================================================================
uint8_t read_MPU_ID(I2C_HandleTypeDef *I2Cx);

void MPU6050_reset_wakeup(I2C_HandleTypeDef *I2Cx);

void MPU6050_Init(I2C_HandleTypeDef *I2Cx);

void MPU6050_Read_Accel(I2C_HandleTypeDef *I2Cx, MPU6050_t *DataStruct);

void MPU6050_Read_Gyro(I2C_HandleTypeDef *I2Cx, MPU6050_t *DataStruct);

void MPU6050_Read_Temp(I2C_HandleTypeDef *I2Cx, MPU6050_t *DataStruct);

void MPU6050_Read_All(I2C_HandleTypeDef *I2Cx, MPU6050_t *DataStruct);

void MPU6050_set_accel_scale(I2C_HandleTypeDef *I2Cx);

void MPU6050_set_gyro_scale(I2C_HandleTypeDef *I2Cx);

void MPU6050_set_sample_rate(I2C_HandleTypeDef *I2Cx);

void MPU6050_self_test(I2C_HandleTypeDef *I2Cx);

double smooth(double *input, double window);

void MPU6050_get_norm(float* p_a_norm, float* p_g_norm);

void MPU6050_init(I2C_HandleTypeDef *I2Cx);

void MPU_calibrate(I2C_HandleTypeDef * I2Cx);

extern uint8_t Data;

extern float a_norm;

extern float g_norm;

static volatile double a_pitch;

static volatile double a_roll;

static volatile double a_yaw;

static volatile double g_pitch;

static volatile double g_roll;

static volatile double g_yaw;

#endif /* __MPU6050_H */
