#pragma once

#include <stdint.h>
#include "spi.h"
#include "imu.h"

void mpu9250Init(imuDev_t* imuDev);
void mpu9250GetAccelSample(imuDev_t* imu, vector3_t *data);
void mpu9250GetGyroSample(imuDev_t* imu, vector3_t *data);

bool ak8963Read(imuDev_t* imu, vector3_t *magData);

#define MPU9250_BIT_RESET       0x80
#define I2C_SLV0_EN             0x80
#define AK8963_MAG_I2C_ADDRESS  0x0C

#define MPU9250_BIT_BYPASS_EN        (1 << 1)
#define MPU9250_BIT_INT_ANYRD_2CLEAR (1 << 4)

#define GYRO_SCALE_2000DPS      (2000.0f / (1 << 15))   // 16.384 dps/lsb scalefactor for 2000dps sensors


// AK8963, mag sensor address
#define AK8963_MAG_I2C_ADDRESS          0x0C
#define AK8963_DEVICE_ID                0x48

// Registers
#define AK8963_MAG_REG_WIA              0x00
#define AK8963_MAG_REG_INFO             0x01
#define AK8963_MAG_REG_ST1              0x02
#define AK8963_MAG_REG_HXL              0x03
#define AK8963_MAG_REG_HXH              0x04
#define AK8963_MAG_REG_HYL              0x05
#define AK8963_MAG_REG_HYH              0x06
#define AK8963_MAG_REG_HZL              0x07
#define AK8963_MAG_REG_HZH              0x08
#define AK8963_MAG_REG_ST2              0x09
#define AK8963_MAG_REG_CNTL1            0x0A
#define AK8963_MAG_REG_CNTL2            0x0B
#define AK8963_MAG_REG_ASCT             0x0C // self test
#define AK8963_MAG_REG_I2CDIS           0x0F
#define AK8963_MAG_REG_ASAX             0x10 // Fuse ROM x-axis sensitivity adjustment value
#define AK8963_MAG_REG_ASAY             0x11 // Fuse ROM y-axis sensitivity adjustment value
#define AK8963_MAG_REG_ASAZ             0x12 // Fuse ROM z-axis sensitivity adjustment value

#define READ_FLAG                       0x80
#define I2C_SLV0_EN                     0x80

#define ST1_DATA_READY                  0x01
#define ST1_DATA_OVERRUN                0x02

#define ST2_MAG_SENSOR_OVERFLOW         0x08

#define CNTL1_MODE_POWER_DOWN           0x00
#define CNTL1_MODE_ONCE                 0x01
#define CNTL1_MODE_CONT1                0x02
#define CNTL1_MODE_CONT2                0x06
#define CNTL1_MODE_SELF_TEST            0x08
#define CNTL1_MODE_FUSE_ROM             0x0F
#define CNTL1_BIT_14_BIT                0x00
#define CNTL1_BIT_16_BIT                0x10

#define CNTL2_SOFT_RESET                0x01

#define I2CDIS_DISABLE_MASK             0x1D
