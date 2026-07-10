#ifndef BARO_HAL_H
#define BARO_HAL_H

#include<stdint.h>
#include<stdbool.h>

typedef struct{
bool (*init)(void);
bool (*read)(float *pressure_pa, float *temperature_c, uint32_t *ts_us);
bool (*healthy)(void);
}baro_dev_t;

extern baro_dev_t baro_mock;

#endif