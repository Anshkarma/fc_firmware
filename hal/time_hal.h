#ifndef TIME_HAL_H
#define TIME_HAL_H

#include <stdint.h>

typedef struct {
    uint32_t (*micros)(void); // for microseconds
    uint32_t (*millis)(void); // for milliseconds
} time_dev_t;

extern time_dev_t time_mock;

#endif // TIME_HAL_H