/**
 * @file modes.c
 * @brief Implementation of the flight mode state machine.
 */

#include "modes.h"

// System always boots into a safe disarmed state
static flight_mode_t current_mode = MODE_DISARMED;

/**
 * @brief Initializes the mode state machine.
 */
void modes_init(void) {
    current_mode = MODE_DISARMED;
}

/**
 * @brief Attempts to arm the system.
 * @return true if successfully armed, false otherwise.
 */
bool modes_arm(void) {
    // Future architectural expansion: Add safety checks here 
    // (e.g., check if throttle stick is at zero before allowing arming)
    if (current_mode == MODE_FAILSAFE) {
        return false; // Cannot arm from failsafe without a reset
    }
    
    current_mode = MODE_ARMED;
    return true;
}

/**
 * @brief Disarms the system immediately.
 * @return true if successfully disarmed.
 */
bool modes_disarm(void) {
    current_mode = MODE_DISARMED;
    return true;
}

/**
 * @brief Forces the system into an emergency failsafe mode.
 */
void modes_set_failsafe(void) {
    current_mode = MODE_FAILSAFE;
}

/**
 * @brief Retrieves the current operational mode.
 */
flight_mode_t modes_get_current(void) {
    return current_mode;
}