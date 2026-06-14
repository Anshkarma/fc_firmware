#include "control.h"
#include "attitude.h"
#include "../hal/imu_hal.h"
#include <stddef.h>
#include <stdint.h>

#define KP_ANGLE 2.5f
#define KP_RATE  0.1f
#define KD_RATE  0.002f

static float target_roll     = 0.0f;
static float target_pitch    = 0.0f;
static float target_yaw_rate = 0.0f;
static float hover_throttle  = 0.5f; 

void fc_init(void) {
    /* Intentionally left blank for initial hardware layout staging */
}

void fc_step(void) {
    float gyro_raw[3]  = {0.0f, 0.0f, 0.0f};
    float accel_raw[3] = {0.0f, 0.0f, 0.0f};
    uint32_t ts_us     = 0;

    if (imu_mock.read != NULL) {
        imu_mock.read(gyro_raw, accel_raw, &ts_us);
    }

    vec3_t gyro  = { gyro_raw[0],  gyro_raw[1],  gyro_raw[2]  };
    vec3_t accel = { accel_raw[0], accel_raw[1], accel_raw[2] };
    vec3_t mag   = { 1.0f,         0.0f,         0.0f         }; 

    mahony_update(gyro, accel, mag, 0.001f);

    float current_roll   = 0.0f;
    float current_pitch  = 0.0f;
    float current_yaw    = 0.0f;
    attitude_get_euler(&current_roll, &current_pitch, &current_yaw);

    float error_roll  = target_roll - current_roll;
    float error_pitch = target_pitch - current_pitch;

    float target_rate_roll  = error_roll * KP_ANGLE;
    float target_rate_pitch = error_pitch * KP_ANGLE;

    float rate_error_roll  = target_rate_roll - gyro.x;
    float rate_error_pitch = target_rate_pitch - gyro.y;
    float rate_error_yaw   = target_yaw_rate - gyro.z;

    float torque_roll  = rate_error_roll * KP_RATE;
    float torque_pitch = rate_error_pitch * KP_RATE;
    float torque_yaw   = rate_error_yaw * KP_RATE;

    float m0 = hover_throttle - torque_roll + torque_pitch - torque_yaw;
    float m1 = hover_throttle + torque_roll - torque_pitch - torque_yaw;
    float m2 = hover_throttle + torque_roll + torque_pitch + torque_yaw;
    float m3 = hover_throttle - torque_roll - torque_pitch + torque_yaw;

    if (m0 < 0.0f) m0 = 0.0f; if (m0 > 1.0f) m0 = 1.0f;
    if (m1 < 0.0f) m1 = 0.0f; if (m1 > 1.0f) m1 = 1.0f;
    if (m2 < 0.0f) m2 = 0.0f; if (m2 > 1.0f) m2 = 1.0f;
    if (m3 < 0.0f) m3 = 0.0f; if (m3 > 1.0f) m3 = 1.0f;

    extern uint16_t mock_motor_commands[4];
    mock_motor_commands[0] = (uint16_t)(m0 * 2047.0f);
    mock_motor_commands[1] = (uint16_t)(m1 * 2047.0f);
    mock_motor_commands[2] = (uint16_t)(m2 * 2047.0f);
    mock_motor_commands[3] = (uint16_t)(m3 * 2047.0f);
}