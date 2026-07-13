/*

@file config.c

@brief The single source of truth for tuning parameters.
*/

#include "config.h"
float INTEGRAL_MAX_LIMIT = 5.0f;
float ALPHA_LPF = 0.25f;  // Keep this at 0.25 to prevent Phase Lag!

// Attitude Loop Gains
float kp_roll_angle    = 4.5f;
float kp_roll_rate     = 0.25f;
float ki_roll_rate     = 0.05f;
float kd_roll_rate     = 0.000f; // Zero keeps motors clean of gyro noise

float kp_pitch_angle   = 4.5f;
float kp_pitch_rate    = 0.25f;
float ki_pitch_rate    = 0.05f;
float kd_pitch_rate    = 0.000f; // Zero keeps motors clean of gyro noise

float kp_yaw_angle     = 6.0f;
float kp_yaw_rate      = 0.20f;
float ki_yaw_rate      = 0.10f;
float kd_yaw_rate      = 0.000f;

float YAW_RATE_SCALE   = 2.5f;

// ============================================================================
// True Cascaded Altitude Hold PID Configuration (Split Outer and Inner Loops)
// ============================================================================

float ALT_MAX_CLIMB_RATE = 1.50f;

float ALT_HOVER_THROTTLE = 0.50f;  // True physical hover point
float ALT_INTEGRAL_LIMIT = 10.0f;

float ALPHA_LPF_ALT      = 0.08f;

/* --- THE FIX: SOFTEN THE ALTITUDE GAINS --- */
float kp_alt_angle        = 1.00f;  
float kp_alt_rate         = 0.15f;  
float ki_alt_rate         = 0.05f; 
float kd_alt_rate         = 0.00f;

float MAG_HARD_IRON_X     =  5.0f;
float MAG_HARD_IRON_Y     = -3.0f;
float MAG_HARD_IRON_Z     =  2.0f;