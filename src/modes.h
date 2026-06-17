/**
 * @file modes.h
 * @brief Flight mode state machine definitions.
 * * Ensures strict safety transitions between Disarmed, Armed, and Failsafe states.
 */

#ifndef MODES_H
#define MODES_H

#include <stdbool.h>

/**
 * @brief Core flight states.
 */
typedef enum {
    MODE_DISARMED,  /**< Motors off, PID reset, safe to handle */
    MODE_ARMED,     /**< Motors active, PID running, dangerous */
    MODE_FAILSAFE   /**< Emergency state (e.g., signal loss) */
} flight_mode_t;

void modes_init(void);
bool modes_arm(void);
bool modes_disarm(void);
void modes_set_failsafe(void);
flight_mode_t modes_get_current(void);

#endif // MODES_H