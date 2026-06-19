/**
 * @file fc_main.c
 * @brief Main flight controller loop. Runs at 1000Hz.
 */

#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include "types.h"
#include "modes.h"
#include "attitude.h"
#include "control.h"
#include "mixer.h"

extern void motor_hal_write(uint16_t frames[4]);
extern uint16_t dshot_encode_frame(uint16_t throttle, bool telemetry);

// Altitude hold state
static float target_altitude = 10.0f;
static float alt_integral = 0.0f;

void fc_init(void) {
    modes_init();
    control_init();
    modes_disarm();
}

void fc_step(vec3_t gyro, vec3_t accel, vec3_t mag, vec3_t sticks, 
             float throttle_stick, float alt_m, float vz_m, float dt) {
    
    // Update attitude estimate
    mahony_update(gyro, accel, mag, dt);
    
    float roll_deg = 0.0f, pitch_deg = 0.0f, yaw_deg = 0.0f;
    attitude_get_euler(&roll_deg, &pitch_deg, &yaw_deg);
    
    // Convert to radians
    vec3_t attitude_rad = {
        roll_deg * (3.14159265f / 180.0f),
        pitch_deg * (3.14159265f / 180.0f),
        yaw_deg * (3.14159265f / 180.0f)
    };
    
    uint16_t output_frames[4] = {0, 0, 0, 0};
    flight_mode_t mode = modes_get_current();
    
    if (mode == MODE_ARMED) {
        float throttle = throttle_stick;
        
        // Altitude hold: enabled when stick near center (0.45-0.55)
        if (throttle_stick > 0.45f && throttle_stick <= 0.55f) {
            float kp = 0.30f;
            float kd = 0.25f;
            float ki = 0.05f;
            
            float error = target_altitude - alt_m;
            alt_integral += error * dt;
            
            // Anti-windup
            if (alt_integral > 0.2f) alt_integral = 0.2f;
            if (alt_integral < -0.2f) alt_integral = -0.2f;
            
            // Hover at 50% throttle + PID corrections
            throttle = 0.5f + (kp * error) + (ki * alt_integral) - (kd * vz_m);
            
            // Limit range
            if (throttle > 0.8f) throttle = 0.8f;
            if (throttle < 0.2f) throttle = 0.2f;
        } else {
            // Manual throttle mode
            target_altitude = alt_m;
            alt_integral = 0.0f;
        }
        
        // Compute control torques
        control_torque_t torque = control_update(sticks, attitude_rad, gyro, dt);
        
        // Mix to motor outputs
        float motors[4] = {0.0f, 0.0f, 0.0f, 0.0f};
        mixer_update(torque, throttle, motors);
        
        // Encode motor commands to DShot frames
        for (int i = 0; i < 4; i++) {
            uint16_t throttle_val = 48 + (uint16_t)roundf(motors[i] * (2047.0f - 48.0f));
            output_frames[i] = dshot_encode_frame(throttle_val, false);
        }
    } else {
        // Disarmed - kill all motors
        for (int i = 0; i < 4; i++) output_frames[i] = 0;
        control_init();
        alt_integral = 0.0f;
    }
    
    motor_hal_write(output_frames);
}
