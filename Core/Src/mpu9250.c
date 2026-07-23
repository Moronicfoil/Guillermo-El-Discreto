#include "mpu9250.h"

#include "stm32f4xx_hal.h"
#include "stdio.h"
#include "math.h"

/*
 * Functions READ and Write Registers
 */

#define MPU9250_READ_BIT (0x80)
#define SPI_TIMEOUT_MS 10

static inline void CS_Low(MPU9250* dev)  { HAL_GPIO_WritePin(dev->csPort, dev->csPin, GPIO_PIN_RESET); }
static inline void CS_High(MPU9250* dev) { HAL_GPIO_WritePin(dev->csPort, dev->csPin, GPIO_PIN_SET); }

HAL_StatusTypeDef MPU9250_ReadRegister(MPU9250* dev, uint8_t reg, uint8_t* data)
{
    uint8_t tx = reg | MPU9250_READ_BIT;
    HAL_StatusTypeDef status;

    CS_Low(dev);
    status = HAL_SPI_Transmit(dev->spiHandle, &tx, 1, SPI_TIMEOUT_MS);
    if(status == HAL_OK)
    {
        status = HAL_SPI_Receive(dev->spiHandle, data, 1, SPI_TIMEOUT_MS);
    }
    CS_High(dev);

    return status;
}

HAL_StatusTypeDef MPU9250_ReadRegisters(MPU9250* dev, uint8_t reg, uint8_t* data, uint8_t lenght)
{
    uint8_t tx = reg | MPU9250_READ_BIT;
    HAL_StatusTypeDef status;

    CS_Low(dev);
    status = HAL_SPI_Transmit(dev->spiHandle, &tx, 1, SPI_TIMEOUT_MS);
    if(status == HAL_OK)
    {
        status = HAL_SPI_Receive(dev->spiHandle, data, lenght, SPI_TIMEOUT_MS);
    }
    CS_High(dev);

    return status;
}

HAL_StatusTypeDef MPU9250_WriteRegister(MPU9250* dev, uint8_t reg, uint8_t data)
{
    uint8_t tx[2];
    tx[0] = reg & 0x7F;   // bit 7 en 0 = escritura
    tx[1] = data;
    HAL_StatusTypeDef status;

    CS_Low(dev);
    status = HAL_SPI_Transmit(dev->spiHandle, tx, 2, SPI_TIMEOUT_MS);
    CS_High(dev);

    return status;
}

/*
 * Functions High Level
 */

HAL_StatusTypeDef MPU9250_Init(MPU9250* dev, SPI_HandleTypeDef* hspi, GPIO_TypeDef* csPort, uint16_t csPin)
{
	dev->spiHandle = hspi;
	dev->csPort = csPort;
	dev->csPin = csPin;

	CS_High(dev);  // CS inactivo por defecto (idle high)
	HAL_Delay(10); // pequeño margen tras power-on

	uint8_t who = 0;

	 dev->gyro_Offset[AXIS_X] = 0.0f;
	 dev->gyro_Offset[AXIS_Y] = 0.0f;
	 dev->gyro_Offset[AXIS_Z] = 0.0f;

	 MPU9250_ReadRegister(dev, WHO_AM_I, &who);
	 if(who != 0x71) return HAL_ERROR;

	 if(MPU9250_WriteRegister(dev, PWR_MGMT_1, 0x00) != HAL_OK) return HAL_ERROR;
	 HAL_Delay(100);

	 if(MPU9250_WriteRegister(dev, CONFIG, 0x01) != HAL_OK) return HAL_ERROR;
	 if(MPU9250_WriteRegister(dev, SMPLRT_DIV, 0x00) != HAL_OK) return HAL_ERROR;
	 if(MPU9250_WriteRegister(dev, GYRO_CONFIG, 0x08) != HAL_OK) return HAL_ERROR;
	 if(MPU9250_WriteRegister(dev, ACCEL_CONFIG, 0x08) != HAL_OK) return HAL_ERROR;
	 if(MPU9250_WriteRegister(dev, ACCEL_CONFIG2, 0x01) != HAL_OK) return HAL_ERROR;

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

