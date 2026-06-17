/**
 * @file control.c
 * @brief Cascade PID Controller implementation for Attitude and Rate stabilization.
 * This module computes the required torque commands (roll, pitch, yaw) based on
 * the error between estimated attitude and target setpoints. It contains no
 * hardware-specific or mixer matrix logic, ensuring complete decoupling.
 */

#include "control.h"
#include "attitude.h"
#include "config.h" // Global tuning parameters (kp, ki, kd, etc.)
#include <math.h>

// ==============================================================================
// INTERNAL STATE (MEMORY REGISTERS)
// ==============================================================================
static float roll_i_accumulator  = 0.0f;
static float pitch_i_accumulator = 0.0f;
static float yaw_i_accumulator   = 0.0f;

static float d_term_filtered_roll  = 0.0f;
static float d_term_filtered_pitch = 0.0f;
static float d_term_filtered_yaw   = 0.0f;


static float last_gyro_roll  = 0.0f;
static float last_gyro_pitch = 0.0f;
static float last_gyro_yaw   = 0.0f;

// ==============================================================================
// CORE ALGORITHM: ADVANCED CASCADE PID ENGINE
// ==============================================================================
/**
 * @brief Computes a single-axis cascade PID control sequence.
 * Executes an outer-loop angle controller producing a target rate, followed by 
 * an inner-loop rate controller generating the final torque command.
 */
static float compute_cascade_pid(float target_angle, float current_angle, float current_rate, 
                                 float kp_angle, float kp_rate, float ki_rate, float kd_rate, 
                                 float *i_accumulator, float *filtered_d, float *last_gyro_meas, float dt) 
{
    // 1. Outer Loop: Angle Error to Target Rate (Strictly Setpoint - Measurement)
    float angle_error = target_angle - current_angle;
    float target_rate = angle_error * kp_angle;
    
    // 2. Inner Loop: Rate Error tracking
    float rate_error = target_rate - current_rate;
    
    // 3. Integral accumulator with anti-windup clamping (§7.6)
    *i_accumulator += rate_error * dt;
    if (*i_accumulator > INTEGRAL_MAX_LIMIT) *i_accumulator = INTEGRAL_MAX_LIMIT;
    if (*i_accumulator < -INTEGRAL_MAX_LIMIT) *i_accumulator = -INTEGRAL_MAX_LIMIT; // ADDED NEGATIVE CLAMP
    
    // 4. Derivative computation on MEASUREMENT to eliminate setpoint-step kicks (§14)
    // Formula: d_meas/dt = (current_gyro - last_gyro) / dt
    float raw_derivative = (current_rate - (*last_gyro_meas)) / dt;
    
    // Apply the configured low-pass filter to mitigate high-frequency noise
    *filtered_d = (ALPHA_LPF * raw_derivative) + ((1.0f - ALPHA_LPF) * (*filtered_d));
    
    // Update the memory register with the current gyro measurement for the next step
    *last_gyro_meas = current_rate; 
    
    // 5. Final torque computation
    // Note: Subtracting the derivative term because damping must oppose the raw physical rate of rotation
    return (rate_error * kp_rate) + ((*i_accumulator) * ki_rate) - ((*filtered_d) * kd_rate);
}

// ==============================================================================
// PUBLIC APIs
// ==============================================================================

/**
 * @brief Resets all PID memory registers. Must be called upon system arming.
 */
void control_init(void) {
    roll_i_accumulator = 0.0f; pitch_i_accumulator = 0.0f; yaw_i_accumulator = 0.0f;
    d_term_filtered_roll = 0.0f; d_term_filtered_pitch = 0.0f; d_term_filtered_yaw = 0.0f;
    last_gyro_roll = 0.0f; last_gyro_pitch = 0.0f; last_gyro_yaw = 0.0f;
}

/**
 * @brief Computes necessary torque vectors based on current sensor state.
 * @param setpoint Desired target angles (Roll, Pitch, Yaw).
 * @param state Estimated current angles (Roll, Pitch, Yaw).
 * @param gyro Raw or filtered gyroscope rates.
 * @param dt Loop time delta in seconds.
 * @return control_torque_t The calculated torque commands to be fed into the mixer.
 */
control_torque_t control_update(vec3_t setpoint, vec3_t state, vec3_t gyro, float dt) 
{
    control_torque_t out_torque;

    out_torque.roll = compute_cascade_pid(setpoint.x, state.x, gyro.x, 
                                          kp_roll_angle, kp_roll_rate, ki_roll_rate, kd_roll_rate, 
                                          &roll_i_accumulator, &d_term_filtered_roll, &last_gyro_roll, dt);

    out_torque.pitch = compute_cascade_pid(setpoint.y, state.y, gyro.y, 
                                           kp_pitch_angle, kp_pitch_rate, ki_pitch_rate, kd_pitch_rate, 
                                           &pitch_i_accumulator, &d_term_filtered_pitch, &last_gyro_pitch, dt);

    out_torque.yaw = compute_cascade_pid(setpoint.z, state.z, gyro.z, 
                                         kp_yaw_angle, kp_yaw_rate, ki_yaw_rate, kd_yaw_rate, 
                                         &yaw_i_accumulator, &d_term_filtered_yaw, &last_gyro_yaw, dt);

    return out_torque;
}