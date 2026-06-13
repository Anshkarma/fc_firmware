#ifndef MOTOR_HAL_H
#define MOTOR_HAL_H

#include <stdint.h>
#include <stdbool.h>

typedef struct {
    bool (*init)(void); 
    bool (*send_dshot)(const uint16_t motor_cmd[4]);
    bool (*arm)(void);
    bool (*disarm)(void);
} motor_dev_t;

extern motor_dev_t motor_mock;

#endif // MOTOR_HAL_H