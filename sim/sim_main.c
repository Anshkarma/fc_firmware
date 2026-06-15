#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <math.h>
#include <io.h>     // Windows matrix handling
#include <process.h>
#include "plant.h"
#include "attitude.h"
#include "..\src\control.h"
#include "..\src\attitude.h"

/* Linkages to simulated hardware endpoints */
extern uint32_t current_sim_time_us;
extern uint16_t mock_motor_commands[4];

/* Quaternion to Euler Deg conversion engine */
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
    uint32_t target_seed = 42;
    char scenario_name[64] = "hover";
    float duration_s = 30.0f;
    float current_disturbance_torque = 0.0f;
    
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

    /* 1. Environment State Configuration Routing */
    quad_state_t initial_state;
    memset(&initial_state, 0, sizeof(quad_state_t));
    initial_state.orientation.w = 1.0f; 

    if (strcmp(scenario_name, "tilt") == 0) {
        float half_angle = 30.0f * (3.14159265f / 180.0f) * 0.5f;
        initial_state.orientation.w = cosf(half_angle);
        initial_state.orientation.x = sinf(half_angle);
        duration_s = 5.0f;
    } else if (strcmp(scenario_name, "disturbance") == 0) {
        disturbance_t dist = { .torque_Nm = {0.05f, 0.0f, 0.0f}, .start_us = 2000000, .duration_us = 50000 };
        plant_inject_disturbance(&dist);
        duration_s = 8.0f;
    }

    plant_init(target_seed, &initial_state);
    current_sim_time_us = 0;
    fc_init();

    /* 2. Create the Logs directory if it doesn't exist (Windows Native Command) */
    system("if not exist logs mkdir logs");

    /* 3. File Handling: Dynamic Path Allocation string formatting */
    char csv_path[128];
    snprintf(csv_path, sizeof(csv_path), "../logs/%s.csv", scenario_name);
    
    FILE *csv_file = fopen(csv_path, "w");
    if (!csv_file) {
        printf("[ERROR] Failed to instantiate target file tracking pointer path: %s\n", csv_path);
        return -1;
    }

    /* 4. Strict Column Requirements Template Injection */
    fprintf(csv_file, "time_s,pos_x,pos_y,pos_z,vel_x,vel_y,vel_z,roll_true,pitch_true,yaw_true,roll_est,pitch_est,yaw_est,rate_roll,rate_pitch,rate_yaw,setpoint_roll,setpoint_pitch,setpoint_yaw,motor1,motor2,motor3,motor4,dshot1,dshot2,dshot3,dshot4,disturbance_torque\n");

    /* 5. Execution Loop (100Hz Logging Factor reduction condition) */
    uint32_t total_steps = (uint32_t)(duration_s * 1000.0f);
    
    for (uint32_t step = 0; step < total_steps; step++) {
        // Disturbance active timeline validation tracking
        if (strcmp(scenario_name, "disturbance") == 0 && current_sim_time_us >= 2000000 && current_sim_time_us < 2050000) {
            current_disturbance_torque = 0.05f;
        } else {
            current_disturbance_torque = 0.0f;
        }

        fc_step();
        plant_step(mock_motor_commands, current_sim_time_us);
        
        const quad_state_t *truth = plant_get_state();
        float t_roll, t_pitch, t_yaw;
        quat_to_euler_deg(truth->orientation, &t_roll, &t_pitch, &t_yaw);

        float e_roll = 0.0f, e_pitch = 0.0f, e_yaw = 0.0f;
        attitude_get_euler(&e_roll, &e_pitch, &e_yaw);
        
        /* 100Hz Downsampling Logger Condition (Write every 10ms subtick) */
        if (step % 10 == 0) {
            float simulated_seconds = (float)current_sim_time_us / 1000000.0f;
            
            // Normalize raw PWM values back to 0.0 - 1.0 range
            float m0_norm = (float)mock_motor_commands[0] / 1000.0f;
            float m1_norm = (float)mock_motor_commands[1] / 1000.0f;
            float m2_norm = (float)mock_motor_commands[2] / 1000.0f;
            float m3_norm = (float)mock_motor_commands[3] / 1000.0f;

            // Generate wire-format 16-bit DShot Hex values
            uint16_t ds0 = (uint16_t)(m0_norm * 2047.0f);
            uint16_t ds1 = (uint16_t)(m1_norm * 2047.0f);
            uint16_t ds2 = (uint16_t)(m2_norm * 2047.0f);
            uint16_t ds3 = (uint16_t)(m3_norm * 2047.0f);

            // Print sequential stream aligned template
            fprintf(csv_file, "%.3f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.4f,%.4f,%.4f,%.4f,0x%03X,0x%03X,0x%03X,0x%03X,%.2f\n",
                    simulated_seconds,
                    0.0f, 0.0f, truth->position.z,             // pos_x, pos_y, pos_z
                    0.0f, 0.0f, truth->velocity.z,      // vel_x, vel_y, vel_z
                    t_roll, t_pitch, t_yaw,                    // Ground Truth Attitude
                    e_roll, e_pitch, e_yaw,                    // Estimator Attitude (Mahony)
                    truth->angular_rate.x * (180.0f/3.14159f), // Gyro readings converted to deg/s
                    truth->angular_rate.y * (180.0f/3.14159f),
                    truth->angular_rate.z * (180.0f/3.14159f),
                    0.0f, 0.0f, 0.0f,                          // Setpoints targets
                    m0_norm, m1_norm, m2_norm, m3_norm,        // Normalized Actuators
                    ds0, ds1, ds2, ds3,                        // 16-bit DShot tracking
                    current_disturbance_torque);               // Non-zero only during injection
        }      
        
        current_sim_time_us += 1000;
    }

    fclose(csv_file);
    printf("[SUCCESS] Telemetry pipeline output flushed directly to: %s\n", csv_path);
    return 0;
}