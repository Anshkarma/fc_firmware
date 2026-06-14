#ifndef ATTITUDE_H
#define ATTITUDE_H

#include <stdint.h>

void fc_init(void);
void fc_step(void);

/* Data extraction pipeline for the telemetry and control logic */
void attitude_get_euler(float *roll, float *pitch, float *yaw);

#endif /* ATTITUDE_H */