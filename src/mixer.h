/**
 * @file mixer.h
 * @brief Quad-X motor mixing matrix and saturation handler.
 */

#ifndef MIXER_H
#define MIXER_H

#include "types.h"

/**
 * @brief Converts PID torque commands and global throttle into individual motor outputs.
 * * @param torque The roll, pitch, and yaw torque commands from the controller.
 * @param throttle_norm Normalized throttle command [0.0, 1.0].
 * @param motors_out Array of 4 floats to store normalized motor outputs [0.0, 1.0].
 * Index 0: FR, 1: BR, 2: BL, 3: FL
 */
void mixer_update(control_torque_t torque, float throttle_norm, float motors_out[4]);

#endif // MIXER_H