#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <math.h>
#include "plant.h"

/* Linkages to our simulated hardware mock drivers */
extern uint32_t current_sim_time_us;
extern uint16_t mock_motor_commands[4];

/* Flight Controller Application Loop Signature (to be built later) */
extern void fc_init(void);
extern void fc_step(void);

/* Quaternion to Euler Utility for CSV Logging */
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
    float duration_s = 30.0f; // Default 30s run for hover
    
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
    initial_state.orientation.w = 1.0f; // Default level quaternion

    if (strcmp(scenario_name, "tilt") == 0) {
        /* Scenario 2: 30 degree initial roll (approx 0.523 rad) */
        float half_angle = 30.0f * (3.14159265f / 180.0f) * 0.5f;
        initial_state.orientation.w = cosf(half_angle);
        initial_state.orientation.x = sinf(half_angle);
        duration_s = 5.0f;
    } else if (strcmp(scenario_name, "disturbance") == 0) {
        /* Scenario 3: Disturbance injection at t = 2.0s */
        disturbance_t dist = { .torque_Nm = {0.1f, 0.0f, 0.0f}, .start_us = 2000000, .duration_us = 50000 };
        plant_inject_disturbance(&dist);
        duration_s = 8.0f;
    }

    /* 4. Initialization Pipeline */
    plant_init(target_seed, &initial_state);
    current_sim_time_us = 0;
    
    /* TODO: fc_init(); will be called here once we build the controller */

    /* 5. Telemetry Logging Setup */
    FILE *csv_file = fopen("sim_output.csv", "w");
    if (!csv_file) {
        printf("[ERROR] Failed to instantiate CSV logging output.\n");
        return -1;
    }
    fprintf(csv_file, "time_s,roll_deg,pitch_deg,yaw_deg\n");

    /* 6. Main Synchronous Integration Loop */
    uint32_t total_steps = (uint32_t)(duration_s * 1000.0f);
    
    for (uint32_t step = 0; step < total_steps; step++) {
        /* A. Run Flight Controller Application Logic */
        /* TODO: fc_step(); will execute here */
        
        /* B. Push the Physics Plant Forward via Mock Actuators */
        plant_step(mock_motor_commands, current_sim_time_us);
        
        /* C. Extract True Telemetry for Validation */
        const quad_state_t *truth = plant_get_state();
        float roll, pitch, yaw;
        quat_to_euler_deg(truth->orientation, &roll, &pitch, &yaw);
        
        /* D. Write Sub-Tick to CSV */
        fprintf(csv_file, "%.3f,%.2f,%.2f,%.2f\n", (current_sim_time_us / 1000000.0f), roll, pitch, yaw);
        
        /* E. Advance Simulated Epoch Clock by 1ms */
        current_sim_time_us += 1000;
    }

    fclose(csv_file);
    printf("[SIM] Execution Terminated. Telemetry flushed to 'sim_output.csv'.\n");
    return 0;
}