/**
 * @file attitude.c
 * @brief Mahony Complementary Filter for Attitude Estimation
 * * FRAME OF REFERENCE CONVENTION:
 * - Earth Frame: NED (North, East, Down). +Z points towards gravity.
 * - Body Frame: Forward-Right-Down (FRD).
 * * QUATERNION CONVENTION:
 * - Hamilton scalar-first: q = [w, x, y, z] where w is the real scalar part.
 * - Represents the rotation from the Earth Frame to the Body Frame.
 */

#include "attitude.h"
#include "imu_hal.h"
#include "mag_hal.h"
#include "motor_hal.h"
#include <math.h>

/* Mahony Filter Tunable Gain Constraints */
#define KP_ACC 2.0f
#define KI_ACC 0.005f
#define KP_MAG 1.0f

#define DT_S 0.001f /* 1 kHz Execution Rate */

/* Custom Vector and Quaternion Structures for Estimator */
typedef struct { float x, y, z; } vec3_t;
typedef struct { float w, x, y, z; } quat_t;

/* Internal State Variables */
static quat_t q = {1.0f, 0.0f, 0.0f, 0.0f}; /* Initial orientation (Level) */
static vec3_t e_int = {0.0f, 0.0f, 0.0f};   /* Integral error accumulator */

/* Output State */
static float estimated_roll_deg = 0.0f;
static float estimated_pitch_deg = 0.0f;
static float estimated_yaw_deg = 0.0f;

/* ==========================================================================
 * Mathematical Primitives (Vector & Quaternion Algebra)
 * ========================================================================== */

static float inv_sqrt(float x) {
    /* Standard fast inverse square root can be used, but standard math is safer for compliance */
    return 1.0f / sqrtf(x);
}

static vec3_t vector_cross(vec3_t a, vec3_t b) {
    vec3_t out = {
        .x = a.y * b.z - a.z * b.y,
        .y = a.z * b.x - a.x * b.z,
        .z = a.x * b.y - a.y * b.x
    };
    return out;
}

static quat_t quat_mult(quat_t q1, quat_t q2) {
    quat_t out = {
        .w = q1.w * q2.w - q1.x * q2.x - q1.y * q2.y - q1.z * q2.z,
        .x = q1.w * q2.x + q1.x * q2.w + q1.y * q2.z - q1.z * q2.y,
        .y = q1.w * q2.y - q1.x * q2.z + q1.y * q2.w + q1.z * q2.x,
        .z = q1.w * q2.z + q1.x * q2.y - q1.y * q2.x + q1.z * q2.w
    };
    return out;
}

static vec3_t quat_rotate(vec3_t v, quat_t q) {
    quat_t v_q = {0.0f, v.x, v.y, v.z};
    quat_t q_conj = {q.w, -q.x, -q.y, -q.z};
    quat_t temp = quat_mult(q, v_q);
    quat_t out_q = quat_mult(temp, q_conj);
    vec3_t out = {out_q.x, out_q.y, out_q.z};
    return out;
}

static vec3_t quat_rotate_inv(vec3_t v, quat_t q) {
    quat_t q_inv = {q.w, -q.x, -q.y, -q.z}; /* Assuming unit quaternion */
    return quat_rotate(v, q_inv);
}

/* ==========================================================================
 * Mahony Filter Execution Engine
 * ========================================================================== */

static void mahony_update(vec3_t gyro, vec3_t accel, vec3_t mag) {
    float norm;
    vec3_t a = accel;
    vec3_t m = mag;

    /* 1. Normalize accelerometer and magnetometer measurements */
    norm = a.x*a.x + a.y*a.y + a.z*a.z;
    if (norm == 0.0f) return; /* Handle NaN singularity */
    norm = inv_sqrt(norm);
    a.x *= norm; a.y *= norm; a.z *= norm;

    norm = m.x*m.x + m.y*m.y + m.z*m.z;
    if (norm > 0.0f) {
        norm = inv_sqrt(norm);
        m.x *= norm; m.y *= norm; m.z *= norm;
    }

    /* 2. Estimate expected 'down' vector in the body frame */
    vec3_t world_down = {0.0f, 0.0f, 1.0f}; /* NED Frame +Z is Down */
    vec3_t v_down_body = quat_rotate_inv(world_down, q);

    /* 3. Estimate magnetic field direction in the body frame */
    vec3_t e_mag = {0.0f, 0.0f, 0.0f};
    if (norm > 0.0f) {
        vec3_t h = quat_rotate(m, q);
        vec3_t b = {sqrtf(h.x*h.x + h.y*h.y), 0.0f, h.z};
        vec3_t w_mag_body = quat_rotate_inv(b, q);
        e_mag = vector_cross(m, w_mag_body);
    }

    /* 4. Error computation via cross product */
    vec3_t e_acc = vector_cross(a, v_down_body);

    /* 5. Combined PI logic */
    vec3_t e = {
        KP_ACC * e_acc.x + KP_MAG * e_mag.x + KI_ACC * e_int.x,
        KP_ACC * e_acc.y + KP_MAG * e_mag.y + KI_ACC * e_int.y,
        KP_ACC * e_acc.z + KP_MAG * e_mag.z + KI_ACC * e_int.z
    };
    
    e_int.x += e_acc.x * DT_S;
    e_int.y += e_acc.y * DT_S;
    e_int.z += e_acc.z * DT_S;

    /* 6. Bias-corrected gyroscope matrix */
    vec3_t omega_corrected = {
        gyro.x + e.x,
        gyro.y + e.y,
        gyro.z + e.z
    };

    /* 7. Integrate Quaternion: q_dot = 0.5 * q \otimes omega_q */
    quat_t omega_q = {0.0f, omega_corrected.x, omega_corrected.y, omega_corrected.z};
    quat_t q_dot = quat_mult(q, omega_q);
    
    q.w += 0.5f * q_dot.w * DT_S;
    q.x += 0.5f * q_dot.x * DT_S;
    q.y += 0.5f * q_dot.y * DT_S;
    q.z += 0.5f * q_dot.z * DT_S;

    /* Normalize resulting quaternion */
    norm = inv_sqrt(q.w*q.w + q.x*q.x + q.y*q.y + q.z*q.z);
    q.w *= norm; q.x *= norm; q.y *= norm; q.z *= norm;

    /* 8. Conversion to Euler Angles for Telemetry and Control */
    float sinr_cosp = 2.0f * (q.w * q.x + q.y * q.z);
    float cosr_cosp = 1.0f - 2.0f * (q.x * q.x + q.y * q.y);
    estimated_roll_deg = atan2f(sinr_cosp, cosr_cosp) * (180.0f / 3.14159265f);

    float sinp = 2.0f * (q.w * q.y - q.z * q.x);
    if (fabsf(sinp) >= 1.0f) {
        estimated_pitch_deg = copysignf(90.0f, sinp);
    } else {
        estimated_pitch_deg = asinf(sinp) * (180.0f / 3.14159265f);
    }

    float siny_cosp = 2.0f * (q.w * q.z + q.x * q.y);
    float cosy_cosp = 1.0f - 2.0f * (q.y * q.y + q.z * q.z);
    estimated_yaw_deg = atan2f(siny_cosp, cosy_cosp) * (180.0f / 3.14159265f);
}

/* ==========================================================================
 * Public Application Interfaces
 * ========================================================================== */

void fc_init(void) {
    imu_mock.init();
    mag_mock.init();
    motor_mock.init();
    motor_mock.arm();
    
    /* Reset state matrix */
    q = (quat_t){1.0f, 0.0f, 0.0f, 0.0f};
    e_int = (vec3_t){0.0f, 0.0f, 0.0f};
}

void fc_step(void) {
    float gyro[3] = {0.0f}, accel[3] = {0.0f}, mag[3] = {0.0f};
    uint32_t ts_us = 0;

    if (imu_mock.read(gyro, accel, &ts_us)) {
        /* Magnetometer typical sample rate constraint can be handled here, 
           assuming synchronous extraction for simulation */
        mag_mock.read(mag, &ts_us);
        
        vec3_t v_gyro  = {gyro[0], gyro[1], gyro[2]};
        vec3_t v_accel = {accel[0], accel[1], accel[2]};
        vec3_t v_mag   = {mag[0], mag[1], mag[2]};

        mahony_update(v_gyro, v_accel, v_mag);
    }
}

void attitude_get_euler(float *roll, float *pitch, float *yaw) {
    if(roll) *roll = estimated_roll_deg;
    if(pitch) *pitch = estimated_pitch_deg;
    if(yaw) *yaw = estimated_yaw_deg;
}