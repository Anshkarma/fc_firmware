#include "baro_hal.h"
#include "../sim/plant.h"

extern uint32_t current_sim_time_us;

static bool mock_baro_init(void) {
    return true;
}

static bool mock_baro_read(float *pressure_pa, float *temperature_c, uint32_t *ts_us) {
    if (!pressure_pa || !temperature_c || !ts_us) return false;

    *ts_us = current_sim_time_us;
    plant_generate_baro(pressure_pa, temperature_c, *ts_us);

    return true;
}

static bool mock_baro_healthy(void) {
    return true;
}

/* Concrete interface structure instance registration */
baro_dev_t baro_mock = {
    .init = mock_baro_init,
    .read = mock_baro_read,
    .healthy = mock_baro_healthy
};