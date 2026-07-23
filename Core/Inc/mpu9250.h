/*
 * mpu9250.h
 *
 *  Created on: Jul 17, 2026
 *      Author: Andres Emilio Ibarra Paz
 */

#ifndef INC_MPU9250_H_
#define INC_MPU9250_H_


#include "stm32f4xx_hal.h"


typedef struct {

	SPI_HandleTypeDef *spiHandle;
	GPIO_TypeDef *csPort;
	uint16_t csPin;

    int16_t gyro_raw_data[3];
    int16_t accel_raw_data[3];

    float gyro[3];		// grados
    float accel[3];		// m/s^2
    float gyro_Offset[3];

    float pitch_acc;
	float roll_acc;

	float pitch;
	float roll;

} MPU9250;

typedef enum
{
    AXIS_X = 0,
    AXIS_Y = 1,
    AXIS_Z = 2
} axis_t;

/*
 * ADDRESS I2C
 */

#define MPU9250_I2C_ADDR_LOW_AD0 (0x68 << 1) 	// stm32 bit shift 7
#define MPU9250_I2C_ADDR_HIGH_AD0 (0X69 << 1)

/*
 * REGISTROS
 */

#define WHO_AM_I (0x75)
#define PWR_MGMT_1 (0x6b)
#define GYRO_CONFIG (0x1b)
#define ACCEL_CONFIG (0x1c)
#define ACCEL_CONFIG2 (0x1d)
#define CONFIG (0x1a)
#define SMPLRT_DIV	(0x19)
#define ACCEL_XOUT_H (0x3b)
#define MPU9250_RAD_TO_DEG 57.2957795f
#define ALPHA 0.98f


/*
 * Functions READ and Write Registers
 */

HAL_StatusTypeDef MPU9250_ReadRegister(MPU9250* dev, uint8_t reg, uint8_t* data);

HAL_StatusTypeDef MPU9250_ReadRegisters(MPU9250* dev, uint8_t reg, uint8_t* data, uint8_t lenght);

HAL_StatusTypeDef MPU9250_WriteRegister(MPU9250* dev, uint8_t reg, uint8_t data);



/*
 * Functions High Level
 */

HAL_StatusTypeDef MPU9250_Init(MPU9250* dev, SPI_HandleTypeDef* hspi, GPIO_TypeDef* csPort, uint16_t csPin);

HAL_StatusTypeDef MPU9250_Read_RawData(MPU9250* dev);

void MPU9250_Convert_toUnits(MPU9250* dev);

HAL_StatusTypeDef MPU9250_GetData(MPU9250* dev);

HAL_StatusTypeDef MPU9250_Calibrate(MPU9250* dev, int16_t samples);

void MPU9250_Update(MPU9250* dev, float dt);


#endif /* INC_MPU9250_H_ */
