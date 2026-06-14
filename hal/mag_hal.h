#ifndef MAG_HAL_H
#define MAG_HAL_H

#include<stdint.h>
#include<stdbool.h>

typedef struct{
bool (*init)(void);
bool (*read)(float  mag_ut[3], uint32_t  * ts_us);
bool (*healthy)(void);
}mag_dev_t;

extern mag_dev_t mag_mock;
// extern mag_dev_t mag_ist8310      <for later , not being used now>

#endif