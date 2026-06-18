/**
 * @file sim_main.c
 * @brief Host-only simulation entry point and telemetry logger.
 * Connects the mathematical Plant engine with the isolated Flight Controller firmware
 * via Hardware Abstraction Layer (HAL) mocks.
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

/* ==============================================================================
 * HAL MOCK HEADERS & EXTERN LINKAGES
 * ============================================================================== */
#include "motor_hal.h"
#include "time_hal.h"

extern void fc_init(void);
extern void fc_step(vec3_t gyro, vec3_t accel, vec3_t mag, vec3_t sticks, float throttle_stick, float alt_m, float vz_m, float dt);
/* Import global states from mock drivers */
extern uint32_t current_sim_time_us;     // Defined in time_mock.c
extern uint16_t mock_motor_commands[4];  // Defined in motor_mock.c

/* ==============================================================================
 * HAL BRIDGE IMPLEMENTATION
 * ============================================================================== */
/**
 * @brief Bridge function to connect firmware's HAL call to our simulated motor driver.
 */
void motor_hal_write(uint16_t frames[4]) {
    // Strictly forward the encoded frames to the motor HAL mock
    motor_mock.send_dshot(frames);
}

/* ==============================================================================
 * UTILITY FUNCTIONS
 * ============================================================================== */
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

/* ==============================================================================
 * MAIN EXECUTABLE ENGINE
 * ============================================================================== */
int main(int argc, char* argv[]) {
    uint32_t target_seed = 42;
    char scenario_name[64] = "hover"; 
    
    // 1. Argument Parsing Matrix
    for (int i = 1; i < argc; i++) {
        if (strcmp(argv[i], "--seed") == 0 && (i + 1) < argc) {
            target_seed = (uint32_t)atoi(argv[i + 1]);
            i++;
        } else if (strcmp(argv[i], "--scenario") == 0 && (i + 1) < argc) {
            strncpy(scenario_name, argv[i + 1], sizeof(scenario_name) - 1);
            i++;
        }
    }

    printf("[SIM] Booting Multi-File Test Bench Engine...\n");

    // 2. Scenario Instantiation
    scenario_config_t config;
    if (!scenario_parse_and_setup(scenario_name, &config)) {
        printf("[FATAL] Unknown scenario: '%s'. Valid options: hover, tilt_recovery, disturbance.\n", scenario_name);
        return -1; 
    }

    // 3. Bootstrapping Systems (Plant, HAL Mocks & Firmware)
    plant_init(target_seed, &config.initial_state);
    if (config.type == SCENARIO_DISTURBANCE) {
        plant_inject_disturbance(&config.disturbance);
    }
    
    current_sim_time_us = 0; // Reset global mock clock
    
    motor_mock.init();       // Initialize motor hardware state
    fc_init();               // Initialize firmware logic
    
    // 4. Logging File System Architecture
    system("if not exist logs mkdir logs"); 

    char csv_path[128];
    snprintf(csv_path, sizeof(csv_path), "logs/%s.csv", scenario_name);
    
    FILE *csv_file = fopen(csv_path, "w");
    if (!csv_file) {
        printf("[ERROR] Failed to instantiate target file tracking pointer path: %s\n", csv_path);
        return -1;
    }

    // Strict 28-column requirement vector
    fprintf(csv_file, "time_s,pos_x,pos_y,pos_z,vel_x,vel_y,vel_z,roll_true,pitch_true,yaw_true,roll_est,pitch_est,yaw_est,rate_roll,rate_pitch,rate_yaw,setpoint_roll,setpoint_pitch,setpoint_yaw,motor1,motor2,motor3,motor4,dshot1,dshot2,dshot3,dshot4,disturbance_torque\n");

    // 5. The 1000Hz Deterministic Execution Loop
    uint32_t total_steps = (uint32_t)(config.duration_s * 1000.0f);
                modes_arm();        // Unlock firmware controller
                motor_mock.arm();   // Unlock hardware driver power distribution
    for (uint32_t step = 0; step < total_steps; step++) {

        // A. Sensor Mocking (Hardware Emulation)
        float g_arr[3], a_arr[3], m_arr[3];
        plant_generate_gyro(g_arr, current_sim_time_us);
        plant_generate_accel(a_arr, current_sim_time_us);
        plant_generate_mag(m_arr, current_sim_time_us);
        
        vec3_t v_gyro  = {g_arr[0], g_arr[1], g_arr[2]};
        vec3_t v_accel = {a_arr[0], a_arr[1], a_arr[2]};
        vec3_t v_mag   = {m_arr[0], m_arr[1], m_arr[2]};
        
        // B. RC Stick Commands
        vec3_t stick_cmds = {0.0f, 0.0f, 0.0f}; 
        float throttle_cmd = 0.50f;  
        float dt = 0.001f;

      /* Execute core flight controller step */
       const quad_state_t *current_physics = plant_get_state();
        fc_step(v_gyro, v_accel, v_mag, stick_cmds, throttle_cmd, current_physics->position.z, current_physics->velocity.z, dt);

        // D. Hardware Physics Execution (Consumes output from motor mock)
        plant_step(mock_motor_commands, current_sim_time_us);
        
        // E. Telemetry Downsampling (100Hz Logging Factor)
        if (step % 10 == 0) {
            float current_disturbance_torque = 0.0f;
            if (config.type == SCENARIO_DISTURBANCE && 
                current_sim_time_us >= config.disturbance.start_us && 
                current_sim_time_us < (config.disturbance.start_us + config.disturbance.duration_us)) {
                current_disturbance_torque = config.disturbance.torque_Nm.x;
            }

            const quad_state_t *truth = plant_get_state();
            float t_roll, t_pitch, t_yaw;
            quat_to_euler_deg(truth->orientation, &t_roll, &t_pitch, &t_yaw);

            float e_roll = 0.0f, e_pitch = 0.0f, e_yaw = 0.0f;
            attitude_get_euler(&e_roll, &e_pitch, &e_yaw);
            
            float simulated_seconds = (float)current_sim_time_us / 1000000.0f;

            // Extract normalized motor positions backwards out of encoded wire frames for logging
            float m1_n = 0.0f, m2_n = 0.0f, m3_n = 0.0f, m4_n = 0.0f;
            uint16_t w_t1 = mock_motor_commands[0] >> 5;
            uint16_t w_t2 = mock_motor_commands[1] >> 5;
            uint16_t w_t3 = mock_motor_commands[2] >> 5;
            uint16_t w_t4 = mock_motor_commands[3] >> 5;
            
            if (w_t1 >= 48) m1_n = (float)(w_t1 - 48) / (2047.0f - 48.0f);
            if (w_t2 >= 48) m2_n = (float)(w_t2 - 48) / (2047.0f - 48.0f);
            if (w_t3 >= 48) m3_n = (float)(w_t3 - 48) / (2047.0f - 48.0f);
            if (w_t4 >= 48) m4_n = (float)(w_t4 - 48) / (2047.0f - 48.0f);

            // Execute print layout map exactly aligning 28 target vectors
            fprintf(csv_file, "%.3f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.4f,%.4f,%.4f,%.4f,0x%04X,0x%04X,0x%04X,0x%04X,%.2f\n",
                    simulated_seconds,
                    0.0f, 0.0f, truth->position.z,            
                    0.0f, 0.0f, truth->velocity.z,      
                    t_roll, t_pitch, t_yaw,                    
                    e_roll, e_pitch, e_yaw,                    
                    truth->angular_rate.x * (180.0f/3.14159f), 
                    truth->angular_rate.y * (180.0f/3.14159f),
                    truth->angular_rate.z * (180.0f/3.14159f),
                    stick_cmds.x, stick_cmds.y, stick_cmds.z,                 
                    m1_n, m2_n, m3_n, m4_n,
                    mock_motor_commands[0], mock_motor_commands[1], 
                    mock_motor_commands[2], mock_motor_commands[3], 
                    current_disturbance_torque);               
        }       
        
        current_sim_time_us += 1000;
    }
    
    /* Ensure hardware is disarmed before exit */
    motor_mock.disarm(); 

    fclose(csv_file);
    printf("[SUCCESS] Telemetry pipeline flushed directly to: %s\n", csv_path);
    return 0;
}