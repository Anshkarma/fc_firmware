#ifndef ATTITUDE_H
#define ATTITUDE_H

#include "types.h"

void mahony_update(vec3_t gyro, vec3_t accel, vec3_t mag, float dt);
quat_t get_attitude_quaternion(void);
void attitude_get_euler(float *roll, float *pitch, float *yaw);

#endif // ATTITUDE_H