#ifndef CONTROL_H
#define CONTROL_H

#include "types.h" // Centralized structures source
#include <stdbool.h>

void control_init(void);

// Existing Attitude Cascade PID
control_torque_t control_update(vec3_t setpoint, vec3_t state, vec3_t gyro, float dt);

// New 7.11 Altitude Cascade Controller (Altitude Error -> Target Climb Rate -> Throttle Adjustment)
float control_update_altitude(float target_alt, float current_alt, float current_vz, float dt, bool alt_hold_active);

#endif // CONTROL_H