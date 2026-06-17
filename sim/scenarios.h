/**
 * @file scenarios.h
 * @brief Test scenario definitions and parser (Section 9 Compliance).
 */

#ifndef SCENARIOS_H
#define SCENARIOS_H

#include <stdbool.h>
#include "plant.h"

// Scenario Type Enumeration
typedef enum {
    SCENARIO_HOVER,
    SCENARIO_TILT_RECOVERY,
    SCENARIO_DISTURBANCE,
    SCENARIO_UNKNOWN
} scenario_type_t;

// Consolidated Configuration Structure
typedef struct {
    scenario_type_t type;
    float duration_s;
    quad_state_t initial_state;
    disturbance_t disturbance;
} scenario_config_t;

/**
 * @brief Parses the string scenario name and outputs the rigid math parameters.
 */
bool scenario_parse_and_setup(const char* scenario_name, scenario_config_t* config_out);

#endif // SCENARIOS_H