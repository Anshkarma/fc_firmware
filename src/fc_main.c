/*
@file fc_main.c

@brief Main flight controller loop. Runs at 1000Hz without simulation state leaks.
*/

#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include "types.h"
#include "modes.h"
#include "attitude.h"
#include "control.h"
#include "mixer.h"
#include "altitude.h"
#include "baro_hal.h"
#include "config.h"

extern void motor_hal_write(uint16_t frames[4]);
extern uint16_t dshot_encode_frame(uint16_t throttle, bool telemetry);

// Local Kalman Filter structure tracking altitude state
static kalman_alt_t altitude_kf;
static bool fc_initialized = false;

// Altitude hold state
static float target_altitude = 10.0f;

// Heading hold target tracker (Integrating pilot's yaw stick)
static float target_heading_rad = 0.0f;

void fc_init(void) {
modes_init();
control_init();
modes_disarm();
fc_initialized = false;
target_heading_rad = 0.0f;
}

void fc_step(vec3_t gyro, vec3_t accel, vec3_t mag, vec3_t sticks,
float throttle_stick, float dt) {

// 1. One-time safe initialization of flight state from raw sensors
if (!fc_initialized) {
    float initial_pressure = 101325.0f;
    float initial_temp = 15.0f; // Dummy temp, unused in modern altitude math
    uint32_t baro_ts = 0;
    
    // Fetch raw baseline read directly from HAL
    if (baro_mock.healthy()) {
        baro_mock.read(&initial_pressure, &initial_temp, &baro_ts);
    }
    
    float start_alt = baro_to_altitude(initial_pressure,initial_temp);
    altitude_init(&altitude_kf, start_alt);
    control_init();
    

    
    // Lock initial heading reference from current attitude
    mahony_update(gyro, accel, mag, dt);
    float r_deg = 0.0f, p_deg = 0.0f, y_deg = 0.0f;
    attitude_get_euler(&r_deg, &p_deg, &y_deg);
    target_heading_rad = y_deg * (3.14159265f / 180.0f);
    
    fc_initialized = true;
}

// 2. Read Barometer internally within the FC firmware boundary
float raw_pressure_pa = 101325.0f;
float raw_temp_c = 15.0f;
uint32_t baro_ts = 0;

if (baro_mock.healthy()) {
    baro_mock.read(&raw_pressure_pa, &raw_temp_c, &baro_ts);
}

// 3. Update Kalman Estimator (Convert Raw Pressure -> Altitude -> 1D Filter State)
float raw_alt = baro_to_altitude(raw_pressure_pa, raw_temp_c);
kalman_update(&altitude_kf, raw_alt, dt);

float est_altitude = altitude_kf.z;
float est_climb_rate = altitude_kf.v;

// 4. Update Attitude estimations (Mahony Core)
mahony_update(gyro, accel, mag, dt);

float roll_deg = 0.0f, pitch_deg = 0.0f, yaw_deg = 0.0f;
attitude_get_euler(&roll_deg, &pitch_deg, &yaw_deg);

// Convert eulers to radians for cascade PID inputs
vec3_t attitude_rad = {
    roll_deg * (3.14159265f / 180.0f),
    pitch_deg * (3.14159265f / 180.0f),
    yaw_deg * (3.14159265f / 180.0f)
};

// Integrate pilot's yaw rate stick into a locked absolute target heading reference
target_heading_rad += sticks.z * YAW_RATE_SCALE * dt;

// Wrap integrated heading inside the [-pi, pi] boundary
if (target_heading_rad > 3.14159265f) target_heading_rad -= 2.0f * 3.14159265f;
if (target_heading_rad < -3.14159265f) target_heading_rad += 2.0f * 3.14159265f;

// Map sticks to clean setpoint structures
vec3_t attitude_setpoint = {
    sticks.x,            // Roll target angle
    sticks.y,            // Pitch target angle
    target_heading_rad   // Heading Hold absolute target reference!
};

uint16_t output_frames[4] = {0, 0, 0, 0};
flight_mode_t mode = modes_get_current();

if (mode == MODE_ARMED) {
    float throttle = throttle_stick;
    bool alt_hold_active = false;
    
    // Altitude hold check: triggers when stick is centered (0.45f - 0.55f)
    if (throttle_stick > 0.45f && throttle_stick <= 0.55f) {
        alt_hold_active = true;
        
        // Execute decoupled altitude control loop safely separated in control.c
        float alt_correction = control_update_altitude(target_altitude, est_altitude, est_climb_rate, dt, alt_hold_active);
        
        // Final throttle combines Hover Feedforward Offset + Correction
        throttle = ALT_HOVER_THROTTLE + alt_correction;
        
        // Enforce tight safety limits
        if (throttle > 0.80f) throttle = 0.80f;
        if (throttle < 0.20f) throttle = 0.20f;
    } else {
        // Manual throttle mode resets integration tracker and tracks current altitude
        target_altitude = est_altitude;
        control_update_altitude(target_altitude, est_altitude, est_climb_rate, dt, alt_hold_active);
    }
    
    // Compute attitude correction torques (using integrated absolute setpoints)
    control_torque_t torque = control_update(attitude_setpoint, attitude_rad, gyro, dt);
    
    // Mix inputs to raw motor dynamics
    float motors[4] = {0.0f, 0.0f, 0.0f, 0.0f};
    mixer_update(torque, throttle, motors);
    
    // Map normalized motors [0.0f - 1.0f] to 11-bit DShot frames
    for (int i = 0; i < 4; i++) {
        uint16_t throttle_val = 48 + (uint16_t)roundf(motors[i] * (2047.0f - 48.0f));
        output_frames[i] = dshot_encode_frame(throttle_val, false);
    }
} else {
    // Disarmed - kill and clean states
    for (int i = 0; i < 4; i++) output_frames[i] = 0;
    control_init();
    target_altitude = est_altitude;
    target_heading_rad = attitude_rad.z; // Track current yaw so arming is smooth
}

motor_hal_write(output_frames);


}