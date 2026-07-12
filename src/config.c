/**
 * @file config.c
 * @brief The single source of truth for tuning parameters.
 */

#include "config.h"
float INTEGRAL_MAX_LIMIT = 5.0f;
float ALPHA_LPF = 0.05f;  // Low-pass filter for D-term

// Attitude Loop Gains
float kp_roll_angle  =  14.5f;
float kp_roll_rate     = 0.65f;
float ki_roll_rate     = 0.25f;
float kd_roll_rate     = 0.000f;

float kp_pitch_angle   =  14.5f;
float kp_pitch_rate    = 0.65f;
float ki_pitch_rate    = 0.25f;
float kd_pitch_rate    = 0.000f;

float kp_yaw_angle     = 6.0f;
float kp_yaw_rate      = 0.20f;
float ki_yaw_rate      = 0.10f;
float kd_yaw_rate      = 0.000f;  

float YAW_RATE_SCALE   =  2.5f;     // Max yaw rotation rate in radians per second

// ============================================================================
// True Cascaded Altitude Hold PID Configuration (Split Outer and Inner Loops)
// ============================================================================

float ALT_MAX_CLIMB_RATE = 1.50f;  // Max allowed target climb rate (m/s)


float ALT_HOVER_THROTTLE = 0.55f;  // Baseline throttle feedforward offset
float ALT_INTEGRAL_LIMIT = 1.50f;  // Anti-windup limit for altitude velocity integrator
float ALPHA_LPF_ALT      = 0.20f;  // Low-pass filter for vertical derivative

float kp_alt_angle        = 0.50f;  // Outer Loop P-gain (Softened tracking)
float kp_alt_rate         = 0.15f;  // Inner Loop P-gain (Reduced from 0.40f to stop rocket jumps)
float ki_alt_rate         = 0.03f;  // Inner Loop I-gain
float kd_alt_rate         = 0.00f;  // Keep exactly 0.0f to stop D-term jitter





