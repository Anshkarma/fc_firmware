/**
 * @file control.c
 * @brief Cascade PID controller for attitude and rate stabilization.
 */

#include "control.h"
#include "attitude.h"
#include "config.h"
#include <math.h>

static float roll_i_accumulator = 0.0f;
static float pitch_i_accumulator = 0.0f;
static float yaw_i_accumulator = 0.0f;

static float d_term_filtered_roll = 0.0f;
static float d_term_filtered_pitch = 0.0f;
static float d_term_filtered_yaw = 0.0f;

static float last_gyro_roll = 0.0f;
static float last_gyro_pitch = 0.0f;
static float last_gyro_yaw = 0.0f;

/**
 * Cascade PID: outer loop computes target rate from angle error,
 * inner loop computes torque from rate error.
 */
static float compute_cascade_pid(float target_angle, float current_angle, float current_rate, 
                                 float kp_angle, float kp_rate, float ki_rate, float kd_rate, 
                                 float *i_accumulator, float *filtered_d, float *last_gyro_meas, float dt) 
{
    // Outer loop: angle error -> target rate
    float angle_error = target_angle - current_angle;
    float target_rate = angle_error * kp_angle;
    
    // Inner loop: rate error
    float rate_error = target_rate - current_rate;
    
    // Accumulate integral with limits
    *i_accumulator += rate_error * dt;
    if (*i_accumulator > INTEGRAL_MAX_LIMIT) *i_accumulator = INTEGRAL_MAX_LIMIT;
    if (*i_accumulator < -INTEGRAL_MAX_LIMIT) *i_accumulator = -INTEGRAL_MAX_LIMIT;
    
    // Derivative on measurement (avoids kick from setpoint changes)
    float raw_d = (current_rate - (*last_gyro_meas)) / dt;
    
    // Low-pass filter to reduce noise
    *filtered_d = (ALPHA_LPF * raw_d) + ((1.0f - ALPHA_LPF) * (*filtered_d));
    *last_gyro_meas = current_rate;
    
    // PID output
    return (rate_error * kp_rate) + ((*i_accumulator) * ki_rate) - ((*filtered_d) * kd_rate);
}

void control_init(void) {
    roll_i_accumulator = 0.0f;
    pitch_i_accumulator = 0.0f;
    yaw_i_accumulator = 0.0f;
    d_term_filtered_roll = 0.0f;
    d_term_filtered_pitch = 0.0f;
    d_term_filtered_yaw = 0.0f;
    last_gyro_roll = 0.0f;
    last_gyro_pitch = 0.0f;
    last_gyro_yaw = 0.0f;
}

control_torque_t control_update(vec3_t setpoint, vec3_t state, vec3_t gyro, float dt) 
{
    control_torque_t out;

    out.roll = compute_cascade_pid(setpoint.x, state.x, gyro.x, 
                                   kp_roll_angle, kp_roll_rate, ki_roll_rate, kd_roll_rate, 
                                   &roll_i_accumulator, &d_term_filtered_roll, &last_gyro_roll, dt);

    out.pitch = compute_cascade_pid(setpoint.y, state.y, gyro.y, 
                                    kp_pitch_angle, kp_pitch_rate, ki_pitch_rate, kd_pitch_rate, 
                                    &pitch_i_accumulator, &d_term_filtered_pitch, &last_gyro_pitch, dt);

    out.yaw = compute_cascade_pid(setpoint.z, state.z, gyro.z, 
                                  kp_yaw_angle, kp_yaw_rate, ki_yaw_rate, kd_yaw_rate, 
                                  &yaw_i_accumulator, &d_term_filtered_yaw, &last_gyro_yaw, dt);

    return out;
}
