#include "imu_hal.h"
#include "../sim/plant.h"

extern uint32_t current_sim_time_us;

static bool mock_imu_init(void) {
    return true;
}

static bool mock_imu_read(float gyro_rad_s[3], float accel_m_s2[3], uint32_t *ts_us) {
    if (!gyro_rad_s || !accel_m_s2 || !ts_us) return false;
    
    *ts_us = current_sim_time_us;
    plant_generate_gyro(gyro_rad_s, *ts_us);
    plant_generate_accel(accel_m_s2, *ts_us);
    
    return true;
}

static bool mock_imu_healthy(void) {
    return true;
}

/* Concrete interface structure instance registration */
imu_dev_t imu_mock = {
    .init = mock_imu_init,
    .read = mock_imu_read,
    .healthy = mock_imu_healthy
};