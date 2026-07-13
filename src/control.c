/**
 * @file control.c
 * @brief Cascade PID controller with True Conditional Anti-Windup.
 */

#include "control.h"
#include "config.h"
#include <math.h>

// Static state variables
static float roll_i_accumulator = 0.0f;
static float pitch_i_accumulator = 0.0f;
static float yaw_i_accumulator = 0.0f;
static float d_term_filtered_roll = 0.0f, d_term_filtered_pitch = 0.0f, d_term_filtered_yaw = 0.0f;
static float last_gyro_roll = 0.0f, last_gyro_pitch = 0.0f, last_gyro_yaw = 0.0f;
static float alt_i_accumulator = 0.0f, d_term_filtered_vz = 0.0f, last_vz_meas = 0.0f;

static float compute_cascade_pid(float target_angle, float current_angle, float current_rate, 
                                 float kp_angle, float kp_rate, float ki_rate, float kd_rate, 
                                 float *i_accumulator, float *filtered_d, float *last_gyro_meas, float dt) 
{
    float angle_error = target_angle - current_angle;
    if (angle_error > 3.14159265f) angle_error -= 6.2831853f;
    if (angle_error < -3.14159265f) angle_error += 6.2831853f;

    float target_rate = angle_error * kp_angle;
    float rate_error = target_rate - current_rate;
    
    float raw_d = (current_rate - (*last_gyro_meas)) / dt;
    *filtered_d = (ALPHA_LPF * raw_d) + ((1.0f - ALPHA_LPF) * (*filtered_d));
    *last_gyro_meas = current_rate;
    
    float p_term = rate_error * kp_rate;
    float d_term = (*filtered_d) * kd_rate;

    // Standard Anti-windup
    *i_accumulator += rate_error * dt;
    if (*i_accumulator > INTEGRAL_MAX_LIMIT) *i_accumulator = INTEGRAL_MAX_LIMIT;
    if (*i_accumulator < -INTEGRAL_MAX_LIMIT) *i_accumulator = -INTEGRAL_MAX_LIMIT;
    
    return p_term + ((*i_accumulator) * ki_rate) - d_term;
}

void control_init(void) {
    roll_i_accumulator = pitch_i_accumulator = yaw_i_accumulator = 0.0f;
    d_term_filtered_roll = d_term_filtered_pitch = d_term_filtered_yaw = 0.0f;
    last_gyro_roll = last_gyro_pitch = last_gyro_yaw = 0.0f;
    alt_i_accumulator = 0.0f;
    d_term_filtered_vz = 0.0f;
    last_vz_meas = 0.0f;
}

control_torque_t control_update(vec3_t setpoint, vec3_t state, vec3_t gyro, float dt) {
    control_torque_t out;
    out.roll = compute_cascade_pid(setpoint.x, state.x, gyro.x, kp_roll_angle, kp_roll_rate, ki_roll_rate, kd_roll_rate, &roll_i_accumulator, &d_term_filtered_roll, &last_gyro_roll, dt);
    out.pitch = compute_cascade_pid(setpoint.y, state.y, gyro.y, kp_pitch_angle, kp_pitch_rate, ki_pitch_rate, kd_pitch_rate, &pitch_i_accumulator, &d_term_filtered_pitch, &last_gyro_pitch, dt);
    out.yaw = compute_cascade_pid(setpoint.z, state.z, gyro.z, kp_yaw_angle, kp_yaw_rate, ki_yaw_rate, kd_yaw_rate, &yaw_i_accumulator, &d_term_filtered_yaw, &last_gyro_yaw, dt);
    return out;
}

float control_update_altitude(float target_alt, float current_alt, float current_vz, float dt, bool alt_hold_active) {
    if (!alt_hold_active) {
        alt_i_accumulator = 0.0f;
        last_vz_meas = current_vz;
        return 0.0f;
    }

    float alt_error = target_alt - current_alt;
    float target_vz = alt_error * kp_alt_angle; 
    if (target_vz > ALT_MAX_CLIMB_RATE) target_vz = ALT_MAX_CLIMB_RATE;
    if (target_vz < -ALT_MAX_CLIMB_RATE) target_vz = -ALT_MAX_CLIMB_RATE;

    float vz_error = target_vz - current_vz;
    
    // D-Term logic
    float raw_d_vz = (current_vz - last_vz_meas) / dt;
    d_term_filtered_vz = (ALPHA_LPF_ALT * raw_d_vz) + ((1.0f - ALPHA_LPF_ALT) * d_term_filtered_vz);
    last_vz_meas = current_vz;

    float p_term = vz_error * kp_alt_rate;
    float d_term = d_term_filtered_vz * kd_alt_rate;

    float tentative_i = alt_i_accumulator + (vz_error * dt);
    float tentative_corr = p_term + (tentative_i * ki_alt_rate) - d_term;
    float tentative_throttle = ALT_HOVER_THROTTLE + tentative_corr;

    // Freeze integrator only if throttle hits absolute physical limits (0.0 to 1.0) 
    // AND the error is trying to push it further into saturation.
    bool hitting_upper_limit = (tentative_throttle >= 1.0f) && (vz_error > 0.0f);
    bool hitting_lower_limit = (tentative_throttle <= 0.0f) && (vz_error < 0.0f);

    if (!hitting_upper_limit && !hitting_lower_limit) {
        alt_i_accumulator = tentative_i; // Apply integral
    }

    // Absolute failsafe clamp with huge headroom
    if (alt_i_accumulator > ALT_INTEGRAL_LIMIT) alt_i_accumulator = ALT_INTEGRAL_LIMIT;
    if (alt_i_accumulator < -ALT_INTEGRAL_LIMIT) alt_i_accumulator = -ALT_INTEGRAL_LIMIT;

    return p_term + (alt_i_accumulator * ki_alt_rate) - d_term;
}

