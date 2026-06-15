#ifndef CONTROL_H
#define CONTROL_H

#include "types.h" // Now it gets all structures from the central source

void control_init(void);
control_torque_t control_update(vec3_t setpoint, vec3_t state, vec3_t gyro, float dt);

#endif // CONTROL_H