#ifndef ALTITUDE_H
#define ALTITUDE_H

#include <stdint.h>
#include "types.h"

// Initialize the Kalman filter state
void altitude_init(kalman_alt_t *kf, float initial_alt);

// Convert raw Barometer Pressure and Temperature to raw Altitude (meters)
float baro_to_altitude(float pressure_pa, float temp_c);

// Update Kalman State taking the raw barometer observation
void kalman_update(kalman_alt_t *kf, float raw_alt, float dt);


#endif // ALTITUDE_H

