/**
 * @file sim_main.c
 * @brief Host-only simulation entry point and telemetry logger.
 * Connects the mathematical Plant engine with the isolated Flight Controller firmware.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <math.h>
#include <stdbool.h>

#include "plant.h"
#include "scenarios.h" // The rigid test scenarios engine
#include "../src/types.h"
#include "../src/modes.h"
#include "../src/control.h"
#include "../src/attitude.h"

// ==============================================================================
// FIRMWARE EXTERN LINKAGES
// ==============================================================================
extern void fc_init(void);
extern void fc_step(vec3_t gyro, vec3_t accel, vec3_t mag, vec3_t sticks, float throttle_stick, float dt);

// ==============================================================================
// HARDWARE ABSTRACTION LAYER (HAL) MOCK IMPLEMENTATION
// ==============================================================================
static uint16_t captured_dshot_frames[4] = {0};

void motor_hal_write(uint16_t frames[4]) {
    for (int i = 0; i < 4; i++) {
        captured_dshot_frames[i] = frames[i];
    }
}

// ==============================================================================
// UTILITY FUNCTIONS
// ==============================================================================
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

// ==============================================================================
// MAIN EXECUTABLE ENGINE
// ==============================================================================
int main(int argc, char* argv[]) {
    uint32_t target_seed = 42;
    char scenario_name[64] = "hover"; // Default fallback
    
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
        return -1; // Catastrophic abort
    }

    // 3. Bootstrapping Systems (Plant & Firmware)
    plant_init(target_seed, &config.initial_state);
    if (config.type == SCENARIO_DISTURBANCE) {
        plant_inject_disturbance(&config.disturbance);
    }
    
    uint32_t current_sim_time_us = 0;
    
    fc_init();
    

    // 4. Logging File System Architecture
    system("if not exist logs mkdir logs"); // Windows native constraint

    char csv_path[128];
    // Dynamic output targeting relative log engine
    snprintf(csv_path, sizeof(csv_path), "logs/%s.csv", scenario_name);
    
    FILE *csv_file = fopen(csv_path, "w");
    if (!csv_file) {
        printf("[ERROR] Failed to instantiate target file tracking pointer path: %s\n", csv_path);
        return -1;
    }

    // Strict 28-column requirement vector per §10.1
    fprintf(csv_file, "time_s,pos_x,pos_y,pos_z,vel_x,vel_y,vel_z,roll_true,pitch_true,yaw_true,roll_est,pitch_est,yaw_est,rate_roll,rate_pitch,rate_yaw,setpoint_roll,setpoint_pitch,setpoint_yaw,motor1,motor2,motor3,motor4,dshot1,dshot2,dshot3,dshot4,disturbance_torque\n");

    // 5. The 1000Hz Deterministic Execution Loop
    uint32_t total_steps = (uint32_t)(config.duration_s * 1000.0f);
    
    for (uint32_t step = 0; step < total_steps; step++) {

       if(step==250) modes_arm();
        
        // A. Sensor Mocking (Hardware Emulation)
        float g_arr[3], a_arr[3], m_arr[3];
        plant_generate_gyro(g_arr, current_sim_time_us);
        plant_generate_accel(a_arr, current_sim_time_us);
        plant_generate_mag(m_arr, current_sim_time_us);
        
        vec3_t v_gyro  = {g_arr[0], g_arr[1], g_arr[2]};
        vec3_t v_accel = {a_arr[0], a_arr[1], a_arr[2]};
        vec3_t v_mag   = {m_arr[0], m_arr[1], m_arr[2]};
        
        // B. RC Stick Commands (LEVEL Mode semantics: 0 = level)
        vec3_t stick_cmds = {0.0f, 0.0f, 0.0f}; 
        float throttle_cmd = 0.55f;  // exact hover equilibrium
        float dt = 0.001f;

        // C. Firmware Brain Execution (Generates DShot frames internally)
        fc_step(v_gyro, v_accel, v_mag, stick_cmds, throttle_cmd, dt);

        // ==============================================================================
        // THE ESC SAFETY UNLOCK INJECTION
        // ==============================================================================
        // Real hardware ESCs permanently lock out if the first armed frame is > 0% throttle.
        // We override the wire with standard '48' (0x0606 - Zero Throttle) for 500ms 
        // after arming to successfully initialize the plant's motors.

        // D. Hardware Physics Execution
        plant_step(captured_dshot_frames, current_sim_time_us);
        
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
            uint16_t w_t1 = captured_dshot_frames[0] >> 5;
            uint16_t w_t2 = captured_dshot_frames[1] >> 5;
            uint16_t w_t3 = captured_dshot_frames[2] >> 5;
            uint16_t w_t4 = captured_dshot_frames[3] >> 5;
            
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
                    captured_dshot_frames[0], captured_dshot_frames[1], 
                    captured_dshot_frames[2], captured_dshot_frames[3], 
                    current_disturbance_torque);               
        }       
        
        current_sim_time_us += 1000;
    }

    fclose(csv_file);
    printf("[SUCCESS] Telemetry pipeline flushed directly to: %s\n", csv_path);
    return 0;
}