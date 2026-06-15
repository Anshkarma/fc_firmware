#include "control.h"
#include "math_types.h"
#include "attitude.h"
#include <math.h>
#include <stdint.h>

// ==============================================================================
// 0. EXTERNAL SIMULATOR LINKAGES
// ==============================================================================
extern void plant_generate_gyro(float out_gyro[3], uint32_t ts_us);
extern void plant_generate_accel(float out_accel[3], uint32_t ts_us);
extern void plant_generate_mag(float out_mag[3], uint32_t ts_us);

extern uint32_t current_sim_time_us; 
extern uint16_t mock_motor_commands[4];

typedef struct {
    float m0; float m1; float m2; float m3;
} motor_outputs_t;

// ==============================================================================
// 2. NEW AUTOMATED STABLE GAINS (COMPUTER OPTIMIZED - ZERO EXPLOSION)
// ==============================================================================
// Roll and Pitch share identical physical symmetry
float kp_roll_angle  = 0.062417f; 
float kp_roll_rate   = 0.039812f;  
float kd_roll_rate   = 0.001084f; 
float ki_roll_rate   = 0.001000f; // Minimal structural integral guard

float kp_pitch_angle = 0.062417f;
float kp_pitch_rate  = 0.039812f;
float kd_pitch_rate  = 0.001084f;
float ki_pitch_rate  = 0.001000f;

float kp_yaw_angle   = 0.050000f;
float kp_yaw_rate    = 0.005000f; 
float kd_yaw_rate    = 0.000000f;
float ki_yaw_rate    = 0.000000f;

static const float BASE_THRUST = 0.55f; // Symmetrical float offset

// ==============================================================================
// 3. RUNTIME MEMORY REGISTERS
// ==============================================================================
static float roll_i_accumulator  = 0.0f;
static float pitch_i_accumulator = 0.0f;
static float yaw_i_accumulator   = 0.0f;

#define INTEGRAL_MAX_LIMIT 0.1f 
#define ALPHA_LPF 0.15f 

static float d_term_filtered_roll  = 0.0f;
static float d_term_filtered_pitch = 0.0f;
static float d_term_filtered_yaw   = 0.0f;

static float last_error_roll  = 0.0f;
static float last_error_pitch = 0.0f;
static float last_error_yaw   = 0.0f;

// ==============================================================================
// 4. CORE COMPUTATION PARALLEL ENGINE
// ==============================================================================
static float compute_cascade_pid_advanced(float target_angle, float current_angle, float current_rate, 
                                   float kp_angle, float kp_rate, float ki_rate, float kd_rate, 
                                   float *i_accumulator, float *filtered_d, float *last_rate_error, float dt) 
{
    float angle_error = target_angle - current_angle;
    float target_rate = angle_error * kp_angle;
    float rate_error = target_rate - current_rate;
    
    *i_accumulator += rate_error * dt;
    if (*i_accumulator > INTEGRAL_MAX_LIMIT)  *i_accumulator = INTEGRAL_MAX_LIMIT;
    if (*i_accumulator < -INTEGRAL_MAX_LIMIT) *i_accumulator = -INTEGRAL_MAX_LIMIT;
    
    float raw_derivative = (rate_error - (*last_rate_error)) / dt;
    *filtered_d = (ALPHA_LPF * raw_derivative) + ((1.0f - ALPHA_LPF) * (*filtered_d));
    
    *last_rate_error = rate_error; 
    
    return (rate_error * kp_rate) + ((*i_accumulator) * ki_rate) + ((*filtered_d) * kd_rate);
}

static void control_update(vec3_t estimated_angles, vec3_t raw_gyro_rates, float base_thrust, float dt, motor_outputs_t *motor_out) 
{
    float roll_torque = compute_cascade_pid_advanced(0.0f, estimated_angles.x, raw_gyro_rates.x, 
                                                    kp_roll_angle, kp_roll_rate, ki_roll_rate, kd_roll_rate, 
                                                    &roll_i_accumulator, &d_term_filtered_roll, &last_error_roll, dt);

    float pitch_torque = compute_cascade_pid_advanced(0.0f, estimated_angles.y, raw_gyro_rates.y, 
                                                     kp_pitch_angle, kp_pitch_rate, ki_pitch_rate, kd_pitch_rate, 
                                                     &pitch_i_accumulator, &d_term_filtered_pitch, &last_error_pitch, dt);

    float yaw_torque = compute_cascade_pid_advanced(0.0f, estimated_angles.z, raw_gyro_rates.z, 
                                                   kp_yaw_angle, kp_yaw_rate, ki_yaw_rate, kd_yaw_rate, 
                                                   &yaw_i_accumulator, &d_term_filtered_yaw, &last_error_yaw, dt);

    // Dynamic sign mapping calibrated explicitly with RK4 solver variables inside plant.c
    float m0_raw = base_thrust + roll_torque + pitch_torque + yaw_torque; 
    float m1_raw = base_thrust - roll_torque + pitch_torque - yaw_torque; 
    float m2_raw = base_thrust - roll_torque - pitch_torque + yaw_torque; 
    float m3_raw = base_thrust + roll_torque - pitch_torque - yaw_torque; 

    motor_out->m0 = (m0_raw > 1.0f) ? 1.0f : ((m0_raw < 0.0f) ? 0.0f : m0_raw);
    motor_out->m1 = (m1_raw > 1.0f) ? 1.0f : ((m1_raw < 0.0f) ? 0.0f : m1_raw);
    motor_out->m2 = (m2_raw > 1.0f) ? 1.0f : ((m2_raw < 0.0f) ? 0.0f : m2_raw);
    motor_out->m3 = (m3_raw > 1.0f) ? 1.0f : ((m3_raw < 0.0f) ? 0.0f : m3_raw);
}

// ==============================================================================
// 5. APPLICATION INTERFACES APIs
// ==============================================================================
void fc_init(void) {
    roll_i_accumulator = 0.0f; pitch_i_accumulator = 0.0f; yaw_i_accumulator = 0.0f;
    d_term_filtered_roll = 0.0f; d_term_filtered_pitch = 0.0f; d_term_filtered_yaw = 0.0f;
    last_error_roll = 0.0f; last_error_pitch = 0.0f; last_error_yaw = 0.0f;
}

void fc_step(void) {
    float gyro_arr[3], accel_arr[3], mag_arr[3];
    plant_generate_accel(accel_arr, current_sim_time_us);
    plant_generate_gyro(gyro_arr, current_sim_time_us);
    plant_generate_mag(mag_arr, current_sim_time_us);
    
    vec3_t raw_accel = { accel_arr[0], accel_arr[1], accel_arr[2] };
    vec3_t raw_gyro  = { gyro_arr[0],  gyro_arr[1],  gyro_arr[2] };
    vec3_t raw_mag   = { mag_arr[0],   mag_arr[1],   mag_arr[2] };
    
    float dt = 0.001f; 
    mahony_update(raw_gyro, raw_accel, raw_mag, dt);
    
    float current_roll = 0.0f, current_pitch = 0.0f, current_yaw = 0.0f;
    attitude_get_euler(&current_roll, &current_pitch, &current_yaw);
    vec3_t estimated_vector = {current_roll, current_pitch, current_yaw};
    
    motor_outputs_t output_signals;
    control_update(estimated_vector, raw_gyro, BASE_THRUST, dt, &output_signals);
    
    mock_motor_commands[0] = (uint16_t)(output_signals.m0 * 1000.0f);
    mock_motor_commands[1] = (uint16_t)(output_signals.m1 * 1000.0f);
    mock_motor_commands[2] = (uint16_t)(output_signals.m2 * 1000.0f);
    mock_motor_commands[3] = (uint16_t)(output_signals.m3 * 1000.0f);
}