#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <math.h>
#include "plant.h"
#include "attitude.h"

/* Linkages to our simulated hardware mock drivers */
extern uint32_t current_sim_time_us;
extern uint16_t mock_motor_commands[4];

/* Quaternion to Euler Utility for Ground Truth Validation */
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
    /* 1. Default Configuration Matrices */
    uint32_t target_seed = 42;
    char scenario_name[64] = "hover";
    float duration_s = 30.0f;
    
    /* 2. Command Line Argument Parsing Engine */
    for (int i = 1; i < argc; i++) {
        if (strcmp(argv[i], "--seed") == 0 && (i + 1) < argc) {
            target_seed = (uint32_t)atoi(argv[i + 1]);
            i++;
        } else if (strcmp(argv[i], "--scenario") == 0 && (i + 1) < argc) {
            strncpy(scenario_name, argv[i + 1], sizeof(scenario_name) - 1);
            i++;
        }
    }

    printf("[SIM] Booting Test Bench...\n");
    printf("[SIM] Target Scenario: %s | LCG Seed: %u\n", scenario_name, target_seed);

    /* 3. Scenario Setup Matrix */
    quad_state_t initial_state;
    memset(&initial_state, 0, sizeof(quad_state_t));
    initial_state.orientation.w = 1.0f; 

    if (strcmp(scenario_name, "tilt") == 0) {
        float half_angle = 30.0f * (3.14159265f / 180.0f) * 0.5f;
        initial_state.orientation.w = cosf(half_angle);
        initial_state.orientation.x = sinf(half_angle);
        duration_s = 5.0f;
    } else if (strcmp(scenario_name, "disturbance") == 0) {
        disturbance_t dist = { .torque_Nm = {0.1f, 0.0f, 0.0f}, .start_us = 2000000, .duration_us = 50000 };
        plant_inject_disturbance(&dist);
        duration_s = 8.0f;
    }

    /* 4. Initialization Pipeline */
    plant_init(target_seed, &initial_state);
    current_sim_time_us = 0;
    
    /* Ignite the Flight Controller Brain (Mahony Filter Init) */
    fc_init();

    /* 5. Telemetry Logging Setup (Dual Track: Truth vs Estimate) */
    FILE *csv_file = fopen("sim_output.csv", "w");
    if (!csv_file) {
        printf("[ERROR] Failed to instantiate CSV logging output.\n");
        return -1;
    }
    fprintf(csv_file, "time_s,truth_roll,truth_pitch,truth_yaw,est_roll,est_pitch,est_yaw\n");

    /* 6. Main Synchronous Integration Loop */
    uint32_t total_steps = (uint32_t)(duration_s * 1000.0f);
    
    for (uint32_t step = 0; step < total_steps; step++) {
        /* A. Run Flight Controller Application Logic (Mahony Update) */
        fc_step();
        
        /* B. Push the Physics Plant Forward */
        plant_step(mock_motor_commands, current_sim_time_us);
        
        /* C. Extract True Telemetry */
        const quad_state_t *truth = plant_get_state();
        float t_roll, t_pitch, t_yaw;
        quat_to_euler_deg(truth->orientation, &t_roll, &t_pitch, &t_yaw);

        /* D. Extract Estimated Telemetry from Mahony */
        float e_roll = 0.0f, e_pitch = 0.0f, e_yaw = 0.0f;
        attitude_get_euler(&e_roll, &e_pitch, &e_yaw);
        
        /* E. Write Sub-Tick to CSV */
        fprintf(csv_file, "%.3f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\n", 
                (current_sim_time_us / 1000000.0f), 
                t_roll, t_pitch, t_yaw, 
                e_roll, e_pitch, e_yaw);
        
        /* F. Advance Simulated Epoch Clock by 1ms */
        current_sim_time_us += 1000;
    }

    fclose(csv_file);
    printf("[SIM] Execution Terminated. Telemetry flushed to 'sim_output.csv'.\n");
    return 0;
}