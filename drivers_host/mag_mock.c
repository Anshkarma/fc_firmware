#include "mag_hal.h"
#include "../sim/plant.h"

extern uint32_t current_sim_time_us;

static bool mock_mag_init(void) {
    return true;
}

static bool mock_mag_read(float mag_ut[3], uint32_t *ts_us) {
    if (!mag_ut || !ts_us) return false;
    
    *ts_us = current_sim_time_us;
    plant_generate_mag(mag_ut, *ts_us);
    
    return true;
}

static bool mock_mag_healthy(void) {
    return true;
}

/* Concrete interface structure instance registration */
mag_dev_t mag_mock = {
    .init = mock_mag_init,
    .read = mock_mag_read,
    .healthy = mock_mag_healthy
};