/**
 * @file config.c
 * @brief The single source of truth for tuning parameters.
 */

#include "config.h"

// ==============================================================================
// SYSTEM LIMITS & FILTERS
// ==============================================================================
float INTEGRAL_MAX_LIMIT = 0.5f;  // Normalized output clamp
float ALPHA_LPF          = 0.15f; // Low-pass filter constant for D-term noise

// ==============================================================================
// ROLL AXIS GAINS (Symmetrical Quad-X)
// ==============================================================================
float kp_roll_angle = 6.0f;       // Converts angle error (rad) to rate setpoint (rad/s)
float kp_roll_rate  = 0.02f;      // Converts rate error to normalized torque
float ki_roll_rate  = 0.005f;      // Eliminates steady-state offset
float kd_roll_rate  = 0.000f;     // Dampens high-frequency oscillation

// ==============================================================================
// PITCH AXIS GAINS (Symmetrical Quad-X)
// ==============================================================================
float kp_pitch_angle = 6.0f;
float kp_pitch_rate  = 0.02f;
float ki_pitch_rate  = 0.005f;
float kd_pitch_rate  = 0.000f;

// ==============================================================================
// YAW AXIS GAINS
// ==============================================================================
float kp_yaw_angle = 0.05f;        // Yaw is primarily rate-controlled via sticks directly
float kp_yaw_rate  = 0.01f;       // Needs higher P-gain due to lower aerodynamic authority
float ki_yaw_rate  = 0.000f;
float kd_yaw_rate  = 0.0f;        // Rarely use D-term on Yaw (causes motor twitching)