#include "icm20602.h"
#include "accgyro_mpu.h"
#include "system.h"

#include <stm32h7xx_hal.h>
#include <stdbool.h>

void icm20602Init(imuDev_t* imu)
{
	// 16.4 LSB/(deg/s)
	imu->gyroScale = 16.4;
	imu->acc_1G = 2048;
	imu->accelScale = imu->acc_1G / 9.80665f;

	spiInstance_t *spi = &imu->spiInstance;

	spiSetSpeed(spi, SPI_BAUDRATEPRESCALER_256);

	uint8_t attemptsRemaining = 20;
	while (attemptsRemaining--) {
		uint8_t in = spiBusReadRegister(spi, MPU_RA_WHO_AM_I);
		if (in == ICM20602_WHO_AM_I_CONST) {
			break;
		}
		HAL_Delay(150);
	}

	spiBusWriteRegister(spi, MPU_RA_PWR_MGMT_1, BIT_H_RESET);
	HAL_Delay(100);
	spiBusWriteRegister(spi, MPU_RA_SIGNAL_PATH_RESET, 0x07);
	HAL_Delay(100);

	spiBusWriteRegisterVerify(spi, MPU_RA_USER_CTRL, BIT_I2C_IF_DIS);
	spiBusWriteRegisterVerify(spi, MPU_RA_PWR_MGMT_1, 0);
	spiBusWriteRegisterVerify(spi, MPU_RA_PWR_MGMT_1, INV_CLK_PLL);
	spiBusWriteRegisterVerify(spi, MPU_RA_GYRO_CONFIG, INV_FSR_2000DPS << 3);
	spiBusWriteRegisterVerify(spi, MPU_RA_ACCEL_CONFIG, INV_FSR_16G << 3);
	spiBusWriteRegisterVerify(spi, MPU_RA_CONFIG, 0x00);
	spiBusWriteRegisterVerify(spi, MPU_RA_SMPLRT_DIV, 0x00);
	spiBusWriteRegisterVerify(spi, MPU_RA_INT_PIN_CFG, 1 << 4);
	spiBusWriteRegisterVerify(spi, MPU_RA_USER_CTRL, BIT_I2C_IF_DIS);
	spiBusWriteRegisterVerify(spi, MPU_RA_INT_ENABLE, MPU6500_BIT_RAW_RDY_EN);

	spiSetSpeed(spi, SPI_BAUDRATEPRESCALER_2);
}

void icm20602GetAccelSample(imuDev_t* imu, vector3_t *data)
{
	uint8_t buff[6];

	spiBusReadRegisterBuffer(&imu->spiInstance, MPU_RA_ACCEL_XOUT_H, buff, 6);

	data->x = (int16_t)((buff[0] << 8) | buff[1]);
	data->y = (int16_t)((buff[2] << 8) | buff[3]);
	data->z = (int16_t)((buff[4] << 8) | buff[5]);
}

void icm20602GetGyroSample(imuDev_t* imu, vector3_t *data)
{
	uint8_t buff[7];

	spiBusReadRegisterBuffer(&imu->spiInstance, MPU_RA_GYRO_XOUT_H, buff, 6);

	data->x = (int16_t)((buff[0] << 8) | buff[1]);
	data->y = (int16_t)((buff[2] << 8) | buff[3]);
	data->z = (int16_t)((buff[4] << 8) | buff[5]);
}
