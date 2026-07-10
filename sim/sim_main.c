/**
 * @file sim_main.c
 * @brief Host simulation entry point: drives the firmware against the
 *        physics model and logs telemetry to CSV.
 *
 * All sensor input and motor output crosses through the HAL contracts
 * defined in hal/ (imu_dev_t, mag_dev_t, motor_dev_t) rather than
 * touching the plant model directly. This file is the only place that
 * knows the HAL is backed by a simulation; everything in src/ only
 * ever sees the HAL interface, never the plant.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <math.h>
#include <stdbool.h>

#include "plant.h"
#include "scenarios.h" 
#include "../src/types.h"
#include "../src/modes.h"
#include "../src/control.h"
#include "../src/attitude.h"

#include "imu_hal.h"
#include "mag_hal.h"
#include "motor_hal.h"
#include "time_hal.h"

extern void fc_init(void);
extern void fc_step(vec3_t gyro, vec3_t accel, vec3_t mag, vec3_t sticks, float throttle_stick, float alt_m, float vz_m, float dt);
extern uint32_t current_sim_time_us;
extern uint16_t mock_motor_commands[4];

/**
 * HAL contract implementation (host build): forwards encoded DShot
 * frames from the firmware to the motor HAL. fc_main.c calls this
 * function with no knowledge of what's on the other side of it.
 */
void motor_hal_write(uint16_t frames[4]) {
    motor_mock.send_dshot(frames);
}

/**
 * @brief Converts a quaternion to Euler angles in degrees.
 *
 * Used only for logging ground-truth attitude (roll_true/pitch_true/
 * yaw_true in the CSV) -- the firmware never calls this, it has its
 * own equivalent in attitude.c for the estimated attitude.
 */
static void quat_to_euler_deg(quat_t q, float *roll, float *pitch, float *yaw) {
    float sinr_cosp = 2.0f * (q.w * q.x + q.y * q.z);
    float cosr_cosp = 1.0f - 2.0f * (q.x * q.x + q.y * q.y);
    *roll = atan2f(sinr_cosp, cosr_cosp) * (180.0f / 3.14159265f);
    
    float sinp = 2.0f * (q.w * q.y - q.z * q.x);
    if (fabsf(sinp) >= 1.0f) {
        *pitch = copysignf(90.0f, sinp);
    } else {
        *pitch = asinf(sinp) * (180.0f / 3.14159265f);
    }
    
    float siny_cosp = 2.0f * (q.w * q.z + q.x * q.y);
    float cosy_cosp = 1.0f - 2.0f * (q.y * q.y + q.z * q.z);
    *yaw = atan2f(siny_cosp, cosy_cosp) * (180.0f / 3.14159265f);
}

int main(int argc, char* argv[]) {
    uint32_t seed = 42;
    char scenario[64] = "hover";
    
    // Parse command line arguments
    for (int i = 1; i < argc; i++) {
        if (strcmp(argv[i], "--seed") == 0 && (i + 1) < argc) {
            seed = (uint32_t)atoi(argv[i + 1]);
            i++;
        } else if (strcmp(argv[i], "--scenario") == 0 && (i + 1) < argc) {
            strncpy(scenario, argv[i + 1], sizeof(scenario) - 1);
            i++;
        }
    }

    printf("[SIM] Starting simulation...\n");

    // Setup scenario
    scenario_config_t config;
    if (!scenario_parse_and_setup(scenario, &config)) {
        printf("[ERROR] Unknown scenario: '%s' (hover, tilt_recovery, disturbance)\n", scenario);
        return -1;
    }

    // Initialize simulation components
    plant_init(seed, &config.initial_state);
    if (config.type == SCENARIO_DISTURBANCE) {
        plant_inject_disturbance(&config.disturbance);
    }
    
    current_sim_time_us = 0;
    /* Bring up every HAL device before the firmware starts calling into
     * them. Order doesn't matter here since none depend on each other,
     * but all three must be initialized before the first fc_step(). */
    imu_mock.init();
    mag_mock.init();
    motor_mock.init();
    fc_init();
    
    // Create output directory and open log file
    system("if not exist logs mkdir logs");
    
    char csv_path[128];
    snprintf(csv_path, sizeof(csv_path), "logs/%s.csv", scenario);
    
    FILE *csv_file = fopen(csv_path, "w");
    if (!csv_file) {
        printf("[ERROR] Failed to open output file: %s\n", csv_path);
        return -1;
    }

    // Write CSV header
    fprintf(csv_file, "time_s,pos_x,pos_y,pos_z,vel_x,vel_y,vel_z,roll_true,pitch_true,yaw_true,roll_est,pitch_est,yaw_est,rate_roll,rate_pitch,rate_yaw,setpoint_roll,setpoint_pitch,setpoint_yaw,motor1,motor2,motor3,motor4,dshot1,dshot2,dshot3,dshot4,disturbance_torque\n");

    // Run simulation loop
    uint32_t total_steps = (uint32_t)(config.duration_s * 1000.0f);
    modes_arm();
    motor_mock.arm();
    
    for (uint32_t step = 0; step < total_steps; step++) {
        // Get sensor readings through the HAL (imu_mock/mag_mock internally
        // call plant_generate_gyro/accel/mag -- same noise model, same
        // numbers, just routed through the documented sensor contract).
        float g_arr[3], a_arr[3], m_arr[3];
        uint32_t ts_us;
        imu_mock.read(g_arr, a_arr, &ts_us);
        mag_mock.read(m_arr, &ts_us);
        
        vec3_t gyro = {g_arr[0], g_arr[1], g_arr[2]};
        vec3_t accel = {a_arr[0], a_arr[1], a_arr[2]};
        vec3_t mag = {m_arr[0], m_arr[1], m_arr[2]};
        
        // RC commands (fixed for this test)
        vec3_t sticks = {0.0f, 0.0f, 0.0f};
        float throttle = 0.50f;
        float dt = 0.001f;

        // Run flight controller
        const quad_state_t *state = plant_get_state();
        fc_step(gyro, accel, mag, sticks, throttle, state->position.z, state->velocity.z, dt);

        // Run physics simulation
        plant_step(mock_motor_commands, current_sim_time_us);
        
        // Log data every 10ms (100Hz logging)
        if (step % 10 == 0) {
            float disturbance = 0.0f;
            if (config.type == SCENARIO_DISTURBANCE && 
                current_sim_time_us >= config.disturbance.start_us && 
                current_sim_time_us < (config.disturbance.start_us + config.disturbance.duration_us)) {
                disturbance = config.disturbance.torque_Nm.x;
            }

            const quad_state_t *truth = plant_get_state();
            float t_roll, t_pitch, t_yaw;
            quat_to_euler_deg(truth->orientation, &t_roll, &t_pitch, &t_yaw);

            float e_roll = 0.0f, e_pitch = 0.0f, e_yaw = 0.0f;
            attitude_get_euler(&e_roll, &e_pitch, &e_yaw);
            
            float time_s = (float)current_sim_time_us / 1000000.0f;

            // Decode motor commands from DShot frames
            float m1_n = 0.0f, m2_n = 0.0f, m3_n = 0.0f, m4_n = 0.0f;
            uint16_t throttle_val[4] = {
                mock_motor_commands[0] >> 5,
                mock_motor_commands[1] >> 5,
                mock_motor_commands[2] >> 5,
                mock_motor_commands[3] >> 5
            };
            
            if (throttle_val[0] >= 48) m1_n = (float)(throttle_val[0] - 48) / (2047.0f - 48.0f);
            if (throttle_val[1] >= 48) m2_n = (float)(throttle_val[1] - 48) / (2047.0f - 48.0f);
            if (throttle_val[2] >= 48) m3_n = (float)(throttle_val[2] - 48) / (2047.0f - 48.0f);
            if (throttle_val[3] >= 48) m4_n = (float)(throttle_val[3] - 48) / (2047.0f - 48.0f);

            // Write CSV row
            fprintf(csv_file, "%.3f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.4f,%.4f,%.4f,%.4f,0x%04X,0x%04X,0x%04X,0x%04X,%.2f\n",
                    time_s,
                    truth->position.x, truth->position.y, truth->position.z,                     
                    truth->velocity.x, truth->velocity.y, truth->velocity.z,
                    t_roll, t_pitch, t_yaw,
                    e_roll, e_pitch, e_yaw,
                    truth->angular_rate.x * (180.0f/3.14159f),
                    truth->angular_rate.y * (180.0f/3.14159f),
                    truth->angular_rate.z * (180.0f/3.14159f),
                    sticks.x, sticks.y, sticks.z,
                    m1_n, m2_n, m3_n, m4_n,
                    mock_motor_commands[0], mock_motor_commands[1],
                    mock_motor_commands[2], mock_motor_commands[3],
                    disturbance);
        } // earlier truth posiion on x and y axis were printed hardcoded, 
        
        current_sim_time_us += 1000;
    }
    
    motor_mock.disarm();
    fclose(csv_file);
    printf("[SUCCESS] Log saved to: %s\n", csv_path);
    return 0;
}
