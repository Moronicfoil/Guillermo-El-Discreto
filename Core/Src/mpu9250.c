#include "mpu9250.h"

#include "stm32f4xx_hal.h"
#include "stdio.h"
#include "math.h"

/*
 * Functions READ and Write Registers
 */

HAL_StatusTypeDef MPU9250_ReadRegister(MPU9250* dev, uint8_t reg, uint8_t* data)
{
	return HAL_I2C_Mem_Read(dev->i2cHandle, MPU9250_I2C_ADDR_LOW_AD0, reg, I2C_MEMADD_SIZE_8BIT, data, 1, 100);
}

HAL_StatusTypeDef MPU9250_ReadRegisters(MPU9250* dev, uint8_t reg, uint8_t* data, uint8_t lenght)
{
	return HAL_I2C_Mem_Read(dev->i2cHandle, MPU9250_I2C_ADDR_LOW_AD0, reg, I2C_MEMADD_SIZE_8BIT, data, lenght, 100);
}

HAL_StatusTypeDef MPU9250_WriteRegister(MPU9250* dev, uint8_t reg, uint8_t data)
{
	return HAL_I2C_Mem_Write(dev->i2cHandle,MPU9250_I2C_ADDR_LOW_AD0, reg, I2C_MEMADD_SIZE_8BIT, &data, 1, 100 );
}

/*
 * Functions High Level
 */

HAL_StatusTypeDef MPU9250_Init(MPU9250* dev, I2C_HandleTypeDef* hic2)
{
	dev->i2cHandle = hic2;
	uint8_t who = 0;

	dev->gyro_Offset[AXIS_X] = 0.0f;
	dev->gyro_Offset[AXIS_Y] = 0.0f;
	dev->gyro_Offset[AXIS_Z] = 0.0f;

	MPU9250_ReadRegister(dev, WHO_AM_I, &who);
	if(who != 0x71) return HAL_ERROR;


	if(MPU9250_WriteRegister(dev, PWR_MGMT_1, 0x00) != HAL_OK) return HAL_ERROR;		// Despertamos el sensor

	HAL_Delay(100);

	if(MPU9250_WriteRegister(dev, CONFIG, 0x01) != HAL_OK) return HAL_ERROR;		    // DLPF gyro 184 hz

	if(MPU9250_WriteRegister(dev, SMPLRT_DIV, 0x00) != HAL_OK) return HAL_ERROR;			// 1khz

	if(MPU9250_WriteRegister(dev, GYRO_CONFIG, 0x08) != HAL_OK)	return HAL_ERROR;  		// Configuramos el DPI a 500°/s

	if(MPU9250_WriteRegister(dev, ACCEL_CONFIG, 0x08) != HAL_OK) return HAL_ERROR;		// configuramos  a 4gs

	if(MPU9250_WriteRegister(dev, ACCEL_CONFIG2, 0x01) != HAL_OK) return HAL_ERROR;		// DLPF accel 184hz

	return HAL_OK;

}

HAL_StatusTypeDef MPU9250_Read_RawData(MPU9250* dev)
{
	uint8_t buffer[14];

	if(MPU9250_ReadRegisters(dev, ACCEL_XOUT_H, buffer, 14) != HAL_OK) return HAL_ERROR;

	dev->accel_raw_data[AXIS_X] = (int16_t)((buffer[0] << 8) | buffer[1]);
	dev->accel_raw_data[AXIS_Y] = (int16_t)((buffer[2] << 8) | buffer[3]);
	dev->accel_raw_data[AXIS_Z] = (int16_t)((buffer[4] << 8) | buffer[5]);

	dev->gyro_raw_data[AXIS_X] = (int16_t)((buffer[8] << 8) | buffer[9]);
	dev->gyro_raw_data[AXIS_Y] = (int16_t)((buffer[10] << 8) | buffer[11]);
	dev->gyro_raw_data[AXIS_Z] = (int16_t)((buffer[12] << 8) | buffer[13]);

	return HAL_OK;
}

void MPU9250_Convert_toUnits(MPU9250* dev)
{
	dev->gyro[AXIS_X] = ((float)dev->gyro_raw_data[AXIS_X])/ 65.5f;
	dev->gyro[AXIS_Y] = ((float)dev->gyro_raw_data[AXIS_Y])/ 65.5f;
	dev->gyro[AXIS_Z] = ((float)dev->gyro_raw_data[AXIS_Z])/ 65.5f;

	dev->accel[AXIS_X] = ((float)dev->accel_raw_data[AXIS_X])/ 8192.0f;
	dev->accel[AXIS_Y] = ((float)dev->accel_raw_data[AXIS_Y])/ 8192.0f;
	dev->accel[AXIS_Z] = ((float)dev->accel_raw_data[AXIS_Z])/ 8192.0f;

	dev->gyro[AXIS_X] -= dev->gyro_Offset[AXIS_X];
	dev->gyro[AXIS_Y] -= dev->gyro_Offset[AXIS_Y];
	dev->gyro[AXIS_Z] -= dev->gyro_Offset[AXIS_Z];
}

HAL_StatusTypeDef MPU9250_GetData(MPU9250* dev)
{
	if(MPU9250_Read_RawData(dev) != HAL_OK)
	{
		return HAL_ERROR;
	}

	MPU9250_Convert_toUnits(dev);

	return HAL_OK;
}

HAL_StatusTypeDef MPU9250_Calibrate(MPU9250* dev, int16_t samples)
{
	float offsetx = 0.0f;
	float offsety = 0.0f;
	float offsetz = 0.0f;

	for(int16_t i = 0; i < samples; i++)
	{
		if(MPU9250_GetData(dev) == HAL_OK)
		{
			offsetx += dev->gyro[AXIS_X];
			offsety += dev->gyro[AXIS_Y];
			offsetz += dev->gyro[AXIS_Z];
		}

		HAL_Delay(6);
	}

	dev->gyro_Offset[AXIS_X] = offsetx/(float)samples;
	dev->gyro_Offset[AXIS_Y] = offsety/(float)samples;
	dev->gyro_Offset[AXIS_Z] = offsetz/(float)samples;

	return HAL_OK;
}

void MPU9250_Update(MPU9250* dev, float dt)
{
	dev->roll_acc = atan2f(dev->accel[AXIS_Y], dev->accel[AXIS_Z]) * MPU9250_RAD_TO_DEG;
	dev->pitch_acc = atan2f(-dev->accel[AXIS_X], sqrtf((dev->accel[AXIS_Y] * dev->accel[AXIS_Y])+(dev->accel[AXIS_Z]*dev->accel[AXIS_Z]))) * MPU9250_RAD_TO_DEG;

	dev->roll  = ALPHA * (dev->roll  + dev->gyro[AXIS_X] * dt) + (1.0f - ALPHA) * dev->roll_acc;
	dev->pitch = ALPHA * (dev->pitch + dev->gyro[AXIS_Y] * dt) + (1.0f - ALPHA) * dev->pitch_acc;
}

