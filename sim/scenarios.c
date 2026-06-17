/**
 * @file scenarios.c
 * @brief Implementation of the required Section 9 test scenarios.
 */

#include "scenarios.h"
#include <string.h>
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

bool scenario_parse_and_setup(const char* scenario_name, scenario_config_t* config_out) {
    // 0. Clean slate initialization
    memset(config_out, 0, sizeof(scenario_config_t));
    config_out->initial_state.orientation.w = 1.0f; // Default unrotated quaternion
    config_out->type = SCENARIO_UNKNOWN;

    // 1. SCENARIO 1: STEADY HOVER
    if (strcmp(scenario_name, "hover") == 0) {
        config_out->type = SCENARIO_HOVER;
        config_out->duration_s = 30.0f;
        // Init: pos(0,0,1)m, vel zero, level, rate zero
        config_out->initial_state.position.z = 1.0f; 
        return true;
    } 
    
    // 2. SCENARIO 2: TILT RECOVERY
    else if (strcmp(scenario_name, "tilt_recovery") == 0) {
        config_out->type = SCENARIO_TILT_RECOVERY;
        config_out->duration_s = 5.0f;
        // Init: 30 deg roll. Convert to quaternion (w = cos(theta/2), x = sin(theta/2))
        float half_angle_rad = (30.0f * (M_PI / 180.0f)) * 0.5f;
        config_out->initial_state.orientation.w = cosf(half_angle_rad);
        config_out->initial_state.orientation.x = sinf(half_angle_rad);
        return true;
    }
    
    // 3. SCENARIO 3: DISTURBANCE REJECTION
    else if (strcmp(scenario_name, "disturbance") == 0) {
        config_out->type = SCENARIO_DISTURBANCE;
        config_out->duration_s = 8.0f;
        config_out->initial_state.position.z = 1.0f; // Start at 1m hover
        
        // Inject 0.1 N.m torque on roll axis at t=2.0s for 50ms
        config_out->disturbance.torque_Nm.x = 0.1f;
        config_out->disturbance.start_us = 2000000; 
        config_out->disturbance.duration_us = 50000;
        return true;
    }
    
    // 4. INVALID INPUT
    return false;
}