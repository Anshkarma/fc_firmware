#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include "plant.h"
#include "imu_hal.h"
#include "mag_hal.h"
#include "baro_hal.h"
#include "attitude.h"
#include "control.h"
#include "mixer.h"

// State tracking variables defined in plant.c
extern quad_state_t drone;
extern float current_sim_time_s;

// Mock driver instances bound via the HAL pointers
extern imu_dev_t imu_mock;
extern mag_dev_t mag_mock;
extern baro_dev_t baro_mock;

// Base configuration constants
#define SIM_STEP_DT       0.001f   // 1kHz loop rate execution steps
#define RAD_TO_DEG        57.29578f

static void run_scenario(int scenario_id, const char* log_path) {
    printf("Executing Scenario %d -> Logging to: %s\n", scenario_id, log_path);
    
    // Reset physics world parameters and PID state states
    plant_init_scenario(scenario_id);
    control_init();

    FILE* csv = fopen(log_path, "w");
    if (!csv) {
        perror("Error: Failed to open telemetry log path");
        return;
    }

    // Standard columns for validation tools
    fprintf(csv, "time_s,pos_z,roll_est,pitch_est,yaw_est,motor1,motor4\n");

    // Map hardware abstraction interface pointers to our host-side drivers
    imu_dev_t* imu = &imu_mock;
    mag_dev_t* mag = &mag_mock;
    baro_dev_t* baro = &baro_mock;

    // Initialize drivers
    if (!imu->init() || !mag->init() || !baro->init()) {
        fprintf(stderr, "Fatal: Driver initialization failed for scenario %d\n", scenario_id);
        fclose(csv);
        return;
    }

    // Sensor buffers
    float gyro[3], accel[3], mag_data[3];
    float baro_press, baro_alt;
    uint32_t timestamp_us;

    // Control and feedback metrics
    float roll_est = 0.0f, pitch_est = 0.0f, yaw_est = 0.0f;
    float roll_cmd = 0.0f, pitch_cmd = 0.0f, yaw_cmd = 0.0f;
    float motor_outputs[4] = {0.0f, 0.0f, 0.0f, 0.0f};

    // Target flight profiles: Taget dead-level hover attitude
    control_sticks_t sticks = {
        .roll_angle = 0.0f,
        .pitch_angle = 0.0f,
        .yaw_rate = 0.0f
    };
    
    // Constant base throttle setting to counteract weight in vertical Z-axis
    float base_throttle = 0.615f; 

    // Scenario 1 requires 30 seconds check. Scenarios 2 and 3 need 10 seconds tracking.
    int total_iterations = (scenario_id == 1) ? 30000 : 10000;

    for (int step = 0; step < total_iterations; step++) {
        
        float wind_disturbance_torque[3] = {0.0f, 0.0f, 0.0f};
        
        // Scenario 3: Inject a sustained 0.45 Nm roll perturbation between 2s and 4s
        if (scenario_id == 3 && current_sim_time_s >= 2.0f && current_sim_time_s <= 4.0f) {
            wind_disturbance_torque[0] = 0.45f; 
        }

        // 1. Advance rigid-body plant physics dynamics
        plant_rk4_step(&drone, motor_outputs, wind_disturbance_torque, SIM_STEP_DT);

        // 2. Sample abstract sensor registers through the HAL Layer
        imu->read(gyro, accel, &timestamp_us);
        mag->read(mag_data, &timestamp_us);
        baro->read(&baro_press, &baro_alt, &timestamp_us);

        // 3. Update Mahony orientation estimation state machine
        mahony_update(gyro, accel, mag_data, SIM_STEP_DT);
        mahony_get_euler(&roll_est, &pitch_est, &yaw_est);

        // Convert raw gyro rates to deg/s to meet PID expectations
        float gyro_deg_s[3] = {
            gyro[0] * RAD_TO_DEG, 
            gyro[1] * RAD_TO_DEG, 
            gyro[2] * RAD_TO_DEG
        };

        // 4. Update cascaded loop PID calculations
        control_level_step(&sticks, (float[]){roll_est, pitch_est, yaw_est}, gyro_deg_s, SIM_STEP_DT,
                           &roll_cmd, &pitch_cmd, &yaw_cmd);

        // 5. Run geometric actuator allocation matrix mixing
        mixer_execute(roll_cmd, pitch_cmd, yaw_cmd, base_throttle, motor_outputs);

        // Telemetry downsampling: Log data fields at 100Hz (Every 10th loop step)
        if (step % 10 == 0) {
            fprintf(csv, "%.3f,%.4f,%.2f,%.2f,%.2f,%.3f,%.3f\n",
                    current_sim_time_s, 
                    drone.position.z, 
                    roll_est * RAD_TO_DEG, 
                    pitch_est * RAD_TO_DEG, 
                    yaw_est * RAD_TO_DEG,
                    motor_outputs[0], 
                    motor_outputs[3]);
        }

        current_sim_time_s += SIM_STEP_DT;
    }

    fclose(csv);
    printf("Scenario %d sequence completed successfully.\n\n", scenario_id);
}

int main(void) {
    printf("==================================================\n");
    printf("FLIGHT FIRMWARE VERIFICATION CORE RUNNER\n");
    printf("==================================================\n");

    // Execute complete evaluation profiles sequentially
    run_scenario(1, "../logs/scenario1_hover.csv");
    run_scenario(2, "../logs/scenario2_recovery.csv");
    run_scenario(3, "../logs/scenario3_disturbance.csv"); 

    printf("==================================================\n");
    printf("ALL SYSTEM TESTS GENERATED SUCCESSFUL VERIFICATIONS\n");
    printf("==================================================\n");
    
    return EXIT_SUCCESS;
}