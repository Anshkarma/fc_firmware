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
    
   /* Enforce disarmed state upon system initialization */
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
// Top par globals mein yeh add kar:
static float target_altitude = 10.0f; // Default 1 meter hover target
static float alt_integral = 0.0f;

// Signature update kar (ab isme alt_m aur vz_m bhi aayega)
void fc_step(vec3_t gyro, vec3_t accel, vec3_t mag, vec3_t sticks, float throttle_stick, float alt_m, float vz_m, float dt) {
    // 1. Core State Estimation
    mahony_update(gyro, accel, mag, dt);
    
    float roll_deg = 0.0f, pitch_deg = 0.0f, yaw_deg = 0.0f;
    attitude_get_euler(&roll_deg, &pitch_deg, &yaw_deg);
    
    vec3_t current_attitude = {
        roll_deg * (3.14159265f / 180.0f),
        pitch_deg * (3.14159265f / 180.0f),
        yaw_deg * (3.14159265f / 180.0f)
    };
    
    uint16_t output_frames[4] = {0, 0, 0, 0};
    flight_mode_t current_mode = modes_get_current();
    
    if (current_mode == MODE_ARMED) {
        
        /* ==========================================================
         * ALTITUDE HOLD PID ENGINE 
         * ========================================================== */
        float effective_throttle = throttle_stick;
        
        // if RC throttle in the centre (in between .045 and .055), then altitude hold is being engaged
        if (throttle_stick > 0.45f && throttle_stick <=0.55f) {
            float kp_alt = 0.30f;  // P-gain for height
            float kd_alt = 0.25f;  // D-gain for velocity damping
            float ki_alt = 0.05f;  // I-gain
            
            float error_z = target_altitude - alt_m;
            alt_integral += error_z * dt;
            
            // Limit integral windup
            if (alt_integral > 0.2f) alt_integral = 0.2f;
            if (alt_integral < -0.2f) alt_integral = -0.2f;
            
            // Fundamental Equation: Base Hover (50%) + PID Corrections
            // The D-term uses negative Vz (Measurement Derivative) to prevent kicks
            effective_throttle = 0.5f + (kp_alt * error_z) + (ki_alt * alt_integral) - (kd_alt * vz_m);
            
            // Saturation clamps
            if (effective_throttle > 0.8f) effective_throttle = 0.8f;
            if (effective_throttle < 0.2f) effective_throttle = 0.2f;
        } else {

            // if pilot sends manual throttle
            target_altitude = alt_m;
            alt_integral = 0.0f; // Reset memory
        }
        
        control_torque_t torque = control_update(sticks, current_attitude, gyro, dt);
        
        float motor_norm[4] = {0.0f, 0.0f, 0.0f, 0.0f};
        mixer_update(torque, effective_throttle, motor_norm);
        
        for (int i = 0; i < 4; i++) {
            uint16_t throttle_2047 = 48 + (uint16_t)roundf(motor_norm[i] * (2047.0f - 48.0f));
            output_frames[i] = dshot_encode_frame(throttle_2047, false);
        }
    } else {
        for (int i = 0; i < 4; i++) output_frames[i] = 0;
        control_init(); 
        alt_integral = 0.0f; // Purge Z-axis memory on disarm
    }
    
    motor_hal_write(output_frames);
}