/**
 * @file config.h
 * @brief Centralized configuration and tuning registry.
 * * Contains external declarations for PID gains and filter constants.
 * Prevents multiple-definition linker errors.
 */

#ifndef CONFIG_H
#define CONFIG_H

// System Limits & Filters
extern float INTEGRAL_MAX_LIMIT;
extern float ALPHA_LPF;

// Roll Axis Gains
extern float kp_roll_angle;
extern float kp_roll_rate;
extern float ki_roll_rate;
extern float kd_roll_rate;

// Pitch Axis Gains
extern float kp_pitch_angle;
extern float kp_pitch_rate;
extern float ki_pitch_rate;
extern float kd_pitch_rate;

// Yaw Axis Gains
extern float kp_yaw_angle;
extern float kp_yaw_rate;
extern float ki_yaw_rate;
extern float kd_yaw_rate;

#endif // CONFIG_H