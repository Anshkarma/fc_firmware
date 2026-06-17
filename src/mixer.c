/**
 * @file mixer.c
 * @brief Quad-X mixer with attitude-priority saturation strategy.
 * * Motor Layout (Top-Down View):
 * Motor 3 (FL)      Motor 2 (FR)
 * \               /
 * \             /
 * [CG]
 * /             \
 * /               \
 * Motor 0 (BL)      Motor 1 (BR)
 */

#include "mixer.h"
#include <math.h>

/**
 * @brief Local helper to constrain float values.
 */
static float clampf_local(float val, float min_val, float max_val) {
    if (val < min_val) return min_val;
    if (val > max_val) return max_val;
    return val;
}
void mixer_update(control_torque_t torque, float throttle_norm, float motors_out[4]) {
    float r = torque.roll;
    float p = torque.pitch;
    float y = torque.yaw;
    float t = throttle_norm;

    // REVERSED AXIS SIGN CORRECTION MATRIX TO MATCH PHYSICS INVERSIONS IN PLANT.C
    motors_out[0] = t + r + p + y; 
    motors_out[1] = t - r + p - y; 
    motors_out[2] = t - r - p + y; 
    motors_out[3] = t + r - p - y;

    // Air-Mode Saturation Strategy (Unchanged)
    float max_motor = motors_out[0];
    float min_motor = motors_out[0];
    for (int i = 1; i < 4; i++) {
        if (motors_out[i] > max_motor) max_motor = motors_out[i];
        if (motors_out[i] < min_motor) min_motor = motors_out[i];
    }
    float excess = (max_motor > 1.0f) ? (max_motor - 1.0f) : 0.0f;
    float deficit = (min_motor < 0.0f) ? (0.0f - min_motor) : 0.0f;
    float shift = deficit - excess;
    for (int i = 0; i < 4; i++) {
        motors_out[i] += shift;
        motors_out[i] = clampf_local(motors_out[i], 0.0f, 1.0f);
    }
}