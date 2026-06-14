#ifndef IMU_HAL_H
#define IMU_HAL_H

#include<stdint.h>
#include<stdbool.h>

typedef struct{
bool (*init)(void);
bool (*read)(float  gyro_rad_s[3], float  accel_m_s2[3], uint32_t * ts_us);
bool (*healthy)(void);
}imu_dev_t;

extern imu_dev_t imu_mock;
// extern imu_dev_t imu_mpu6000    <for later , not being used now>

#endif