#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <math.h>
#include "plant.h"
#include "attitude.h"

// Hardware mock declarations local to this execution unit
uint32_t current_sim_time_us = 0;
uint16_t mock_motor_commands[4] = {0, 0, 0, 0};

// Static algorithm variables
static float test_kp_a, test_kp_r, test_kd_r;

// Continuous Attitude Tracking Variables
static float roll_i_accum = 0.0f;
static float last_rate_err_x = 0.0f;

// Pseudo-random float generator for Search Space mapping
static float get_random_coefficient(float min, float max) {
    return min + ((float)rand() / (float)RAND_MAX) * (max - min);
}

// Fixed Symmetrical Quad-X Mixer Target
static void local_mixer_execution(float base, float torque, uint16_t commands[4]) {
    float m0 = base + torque; // Right Side Pairs
    float m1 = base - torque; // Left Side Pairs
    
    // Boundary saturation capping 0 to 1000 PWM Allocation
    commands[0] = (m0 > 1.0f) ? 1000 : ((m0 < 0.0f) ? 0 : (uint16_t)(m0 * 1000.0f));
    commands[1] = (m1 > 1.0f) ? 1000 : ((m1 < 0.0f) ? 0 : (uint16_t)(m1 * 1000.0f));
    commands[2] = commands[1];
    commands[3] = commands[0];
}

// 2-Second Dynamic System Evaluation Window
float evaluate_control_system(float kp_angle, float kp_rate, float kd_rate) {
    quad_state_t init_state;
    memset(&init_state, 0, sizeof(quad_state_t));
    init_state.orientation.w = 1.0f; 
    init_state.orientation.x = 0.174f; // Hard 10-degree initial step disturbance injection

    plant_init(42, &init_state);
    current_sim_time_us = 0;
    roll_i_accum = 0.0f;
    last_rate_err_x = 0.0f;

    float fitness_penalty = 0.0f;

    for (uint32_t step = 0; step < 2000; step++) {
        float gyro_arr[3], accel_arr[3], mag_arr[3];
        plant_generate_accel(accel_arr, current_sim_time_us);
        plant_generate_gyro(gyro_arr, current_sim_time_us);
        plant_generate_mag(mag_arr, current_sim_time_us);

        // Fetch plant dynamic states
        const quad_state_t *current_state = plant_get_state();

        // Cascade Loop Mathematics Execution
        float angle_error = 0.0f - current_state->angular_rate.x; // Simplified feedback tracking
        float target_rate = angle_error * kp_angle;
        float rate_error  = target_rate - current_state->angular_rate.x;

        float d_term = (rate_error - last_rate_err_x) / 0.001f;
        last_rate_err_x = rate_error;

        float target_torque = (rate_error * kp_rate) + (d_term * kd_rate);

        // Run hardware assignment constraints
        local_mixer_execution(0.55f, target_torque, mock_motor_commands);
        plant_step(mock_motor_commands, current_sim_time_us);

        // Penalty tracking bounds
        fitness_penalty += fabsf(current_state->angular_rate.x) + fabsf(current_state->position.z);

        if (isnan(fitness_penalty) || fabsf(current_state->angular_rate.x) > 40.0f) {
            return 999999.0f; // Instability crash constraint met
        }
        current_sim_time_us += 1000;
    }
    return fitness_penalty;
}

int main(void) {
    printf("[AUTOTUNE] Booting Standalone Brute-Force Matrix Generator...\n");
    srand(1337);

    float global_optimum_score = 999999.0f;
    float final_kp_a = 0.0f, final_kp_r = 0.0f, final_kd_r = 0.0f;

    // Search Space loop iteration grid
    for (int loop = 0; loop < 5000; loop++) {
        float test_a = get_random_coefficient(0.01f, 0.40f);
        float test_r = get_random_coefficient(0.001f, 0.05f);
        float test_d = get_random_coefficient(0.0000f, 0.002f);

        float current_score = evaluate_control_system(test_a, test_r, test_d);

        if (current_score < global_optimum_score) {
            global_optimum_score = current_score;
            final_kp_a = test_a;
            final_kp_r = test_r;
            final_kd_r = test_d;
            printf("[STABILITY FOUND] Optimization Match State -> Score: %.2f\n", current_score);
        }
    }

    printf("\n==================================================\n");
    printf("[SUCCESS] AUTOMATED MATHEMATICAL TUNING COMPLETE:\n");
    printf("==================================================\n");
    printf("kp_roll_angle  = %f\n", final_kp_a);
    printf("kp_roll_rate   = %f\n", final_kp_r);
    printf("kd_roll_rate   = %f\n", final_kd_r);
    printf("==================================================\n");

    return 0;
}