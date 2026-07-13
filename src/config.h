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

extern float YAW_RATE_SCALE;


extern float ALT_MAX_CLIMB_RATE;

extern float ALT_HOVER_THROTTLE;
extern float ALT_INTEGRAL_LIMIT;
extern float ALPHA_LPF_ALT;

extern float kp_alt_angle;
extern float kp_alt_rate;
extern float ki_alt_rate;
extern float kd_alt_rate;

/* Hard-iron magnetometer calibration offsets (uT).
 * These are sensor installation constants, not physics constants --
 * they belong here so they can be tuned per-platform without touching
 * the estimator or the simulation plant model. On real hardware these
 * would be measured during a magnetometer calibration routine. */
extern float MAG_HARD_IRON_X;
extern float MAG_HARD_IRON_Y;
extern float MAG_HARD_IRON_Z;

#endif // CONFIG_H