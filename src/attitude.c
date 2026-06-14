#include "attitude.h"
#include <math.h>

#define KP_ACC 2.0f
#define KI_ACC 0.005f

/* Global Filter Tracking Orientation Matrices */
static quat_t q     = {1.0f, 0.0f, 0.0f, 0.0f};  
static vec3_t e_int = {0.0f, 0.0f, 0.0f};    

static vec3_t vector_cross(vec3_t a, vec3_t b) {
    vec3_t res;
    res.x = a.y * b.z - a.z * b.y;
    res.y = a.z * b.x - a.x * b.z;
    res.z = a.x * b.y - a.y * b.x;
    return res;
}

static quat_t quaternion_multiply(quat_t q1, quat_t q2) {
    quat_t res;
    res.w = q1.w*q2.w - q1.x*q2.x - q1.y*q2.y - q1.z*q2.z;
    res.x = q1.w*q2.x + q1.x*q2.w + q1.y*q2.z - q1.z*q2.y;
    res.y = q1.w*q2.y - q1.x*q2.z + q1.y*q2.w + q1.z*q2.x;
    res.z = q1.w*q2.z + q1.x*q2.y - q1.y*q2.x + q1.z*q2.w;
    return res;
}

static vec3_t quaternion_rotate_inverse(vec3_t v, quat_t q_curr) {
    quat_t q_inv = {q_curr.w, -q_curr.x, -q_curr.y, -q_curr.z};
    quat_t v_quat = {0.0f, v.x, v.y, v.z};
    
    quat_t temp = quaternion_multiply(q_inv, v_quat);
    quat_t res_quat = quaternion_multiply(temp, q_curr);
    
    vec3_t res_vec = {res_quat.x, res_quat.y, res_quat.z};
    return res_vec;
}

void mahony_update(vec3_t gyro, vec3_t accel, vec3_t mag, float dt) {
    (void)mag; 
    
    float accel_norm = sqrtf(accel.x*accel.x + accel.y*accel.y + accel.z*accel.z);
    if (accel_norm == 0.0f) return; 
    
    vec3_t a = {accel.x / accel_norm, accel.y / accel_norm, accel.z / accel_norm};
    vec3_t world_down = {0.0f, 0.0f, 1.0f}; 
    vec3_t v_down_body = quaternion_rotate_inverse(world_down, q);

    vec3_t e_acc = vector_cross(a, v_down_body);

    vec3_t e = {
        KP_ACC * e_acc.x,
        KP_ACC * e_acc.y,
        KP_ACC * e_acc.z
    };

    e_int.x += e_acc.x * dt;
    e_int.y += e_acc.y * dt;
    e_int.z += e_acc.z * dt;

    e.x += KI_ACC * e_int.x;
    e.y += KI_ACC * e_int.y;
    e.z += KI_ACC * e_int.z;

    vec3_t omega_corrected = {
        gyro.x + e.x,
        gyro.y + e.y,
        gyro.z + e.z
    };

    quat_t omega_q = {0.0f, omega_corrected.x, omega_corrected.y, omega_corrected.z};
    quat_t q_dot = quaternion_multiply(q, omega_q); 

    q.w += 0.5f * q_dot.w * dt;
    q.x += 0.5f * q_dot.x * dt;
    q.y += 0.5f * q_dot.y * dt;
    q.z += 0.5f * q_dot.z * dt;

    float q_norm = sqrtf(q.w*q.w + q.x*q.x + q.y*q.y + q.z*q.z);
    q.w /= q_norm;
    q.x /= q_norm;
    q.y /= q_norm;
    q.z /= q_norm;
}

void attitude_get_euler(float *roll, float *pitch, float *yaw) {
    *roll = atan2f(2.0f * (q.w * q.x + q.y * q.z), 1.0f - 2.0f * (q.x * q.x + q.y * q.y)) * (180.0f / 3.14159265f);
    
    float sinp = 2.0f * (q.w * q.y - q.z * q.x);
    if (sinp > 1.0f) sinp = 1.0f;
    else if (sinp < -1.0f) sinp = -1.0f;
    *pitch = asinf(sinp) * (180.0f / 3.14159265f);
    
    *yaw = atan2f(2.0f * (q.w * q.z + q.x * q.y), 1.0f - 2.0f * (q.y * q.y + q.z * q.z)) * (180.0f / 3.14159265f);
}

quat_t get_attitude_quaternion(void) {
    return q;
}