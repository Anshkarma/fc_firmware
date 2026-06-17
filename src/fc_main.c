/**
 * @file fc_main.c
 * @brief The core orchestration engine and periodic scheduler.
 * Central nervous system of the flight controller. Handles coordinate transformations,
 * updates orientation states via Mahony filter, triggers cascade PIDs, and commands HAL.
 */

#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include "types.h"
#include "modes.h"
#include "attitude.h"
#include "control.h"
#include "mixer.h"

/* ==============================================================================
 * HARDWARE ABSTRACTION LAYER (HAL) CONTRACTS
 * ============================================================================== */
extern void motor_hal_write(uint16_t frames[4]);
extern uint16_t dshot_encode_frame(uint16_t throttle, bool telemetry);

// ==============================================================================
// FIRMWARE INITIALIZATION
// ==============================================================================

/**
 * @brief Bootstraps all firmware modules upon hardware power-up vector.
 */
void fc_init(void) {
    modes_init();
    control_init();
    
    // Safety paramount: Force system into a disarmed state upon boot execution
    modes_disarm(); 
}

// ==============================================================================
// THE MAIN SCHEDULER TICK (1000Hz Loop)
// ==============================================================================

/**
 * @brief The primary execution loop called by the hardware timer or simulator.
 * @param gyro Raw gyroscope vector in rad/s from plant.
 * @param accel Raw accelerometer vector from plant.
 * @param mag Raw magnetometer vector from plant.
 * @param sticks Commanded target angles from the RC receiver.
 * @param throttle_stick Commanded throttle from the RC receiver [0.0, 1.0].
 * @param dt Delta time since the last execution tick in seconds (0.001f).
 */
void fc_step(vec3_t gyro, vec3_t accel, vec3_t mag, vec3_t sticks, float throttle_stick, float dt) {
    // 1. Core State Estimation: Update the Mahony filter registers
    mahony_update(gyro, accel, mag, dt);
    
    // 2. Fetch Euler Angles from Estimator in Degrees Frame
    float roll_deg = 0.0f;
    float pitch_deg = 0.0f;
    float yaw_deg = 0.0f;
    attitude_get_euler(&roll_deg, &pitch_deg, &yaw_deg);
    
    // 3. Mathematical Domain Alignment: Convert Degrees to Radians for Cascade PID
    vec3_t current_attitude;
    current_attitude.x = roll_deg * (3.14159265f / 180.0f);
    current_attitude.y = pitch_deg * (3.14159265f / 180.0f);
    current_attitude.z = yaw_deg * (3.14159265f / 180.0f);
    
    // 4. Actuator Buffer Vector Allocation
    uint16_t output_frames[4] = {0, 0, 0, 0};
    
    // 5. Read current flight mode from firmware memory state
    flight_mode_t current_mode = modes_get_current();
    
    // 6. Flight State Conditional Matrix Execution
    if (current_mode == MODE_ARMED) {
        // Gyro is already in rad/s from plant—perfect alignment with Radians attitude domain
        control_torque_t torque = control_update(sticks, current_attitude, gyro, dt);
        
        // Zero arbitrary multiplier hacks to enforce linear cascade loop integrity
        float motor_norm[4] = {0.0f, 0.0f, 0.0f, 0.0f};
        mixer_update(torque, throttle_stick, motor_norm);
        
        // Convert normalized floating arrays [0.0, 1.0] to true 16-bit DShot Wire Format
        for (int i = 0; i < 4; i++) {
            uint16_t throttle_2047 = 48 + (uint16_t)roundf(motor_norm[i] * (2047.0f - 48.0f));
            output_frames[i] = dshot_encode_frame(throttle_2047, false);
        }
    } else {
        // Safe Disarmed State: Output clear wire frames to isolate plant physics
        for (int i = 0; i < 4; i++) {
            output_frames[i] = 0;
        }
        
        // Keep PID integration accumulation bounds isolated while disarmed
        control_init(); 
    }
    
    // 7. Hard Boundary Dispatch: Write frames directly to HAL tracking pointer
    motor_hal_write(output_frames);
}