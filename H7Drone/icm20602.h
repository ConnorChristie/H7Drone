#pragma once

#include <stdint.h>
#include "imu.h"

void icm20602Init(imuDev_t* imuDev);
void icm20602GetAccelSample(imuDev_t* imu, vector3_t *data);
void icm20602GetGyroSample(imuDev_t* imu, vector3_t *data);

#define BIT_H_RESET 0x80
#define BIT_I2C_IF_DIS 0x10
#define MPU6500_BIT_RAW_RDY_EN (1 << 0)
