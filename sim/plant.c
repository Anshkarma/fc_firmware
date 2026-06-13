#include "plant.h"
#include <math.h>
#include <string.h>

static quad_state_t  global_state;
static disturbance_t global_disturbance;
static uint32_t      global_seed = 1337;
static vec3_t        gyro_static_bias;

// ============================================================================
// LCG Randomization Algorithm Engine (Section 3.3 Proofed)
// ============================================================================

static uint32_t lcg_step(uint32_t *seed) {
    *seed = *seed * 1664525L + 1013904223L;
    return *seed;
}

static float generate_gaussian(uint32_t *seed, float sigma) {
    float u1, u2, w;
    do {
        u1 = ((float)(lcg_step(seed) & 0xFFFF) / 65535.0f) * 2.0f - 1.0f;
        u2 = ((float)(lcg_step(seed) & 0xFFFF) / 65535.0f) * 2.0f - 1.0f;
        w = u1 * u1 + u2 * u2;
    } while (w >= 1.0f || w == 0.0f);
    
    w = sqrtf((-2.0f * logf(w)) / w);
    return u1 * w * sigma;
}

// ============================================================================
// Hamilton Kinematics Transformations
// ============================================================================

static vec3_t vector_cross(vec3_t a, vec3_t b) {
    vec3_t out = {
        .x = a.y * b.z - a.z * b.y,
        .y = a.z * b.x - a.x * b.z,
        .z = a.x * b.y - a.y * b.x
    };
    return out;
}

static quat_t quaternion_multiply(quat_t q, quat_t r) {
    quat_t out = {
        .w = q.w * r.w - q.x * r.x - q.y * r.y - q.z * r.z,
        .x = q.w * r.x + q.x * r.w + q.y * r.z - q.z * r.y,
        .y = q.w * r.y - q.x * r.z + q.y * r.w + q.z * r.x,
        .z = q.w * r.z + q.x * r.y - q.y * r.x + q.z * r.w
    };
    return out;
}

static vec3_t quaternion_rotate(vec3_t v, quat_t q) {
    quat_t v_q = { 0.0f, v.x, v.y, v.z };
    quat_t q_conj = { q.w, -q.x, -q.y, -q.z };
    quat_t temp = quaternion_multiply(q, v_q);
    quat_t out_q = quaternion_multiply(temp, q_conj);
    vec3_t out = { out_q.x, out_q.y, out_q.z };
    return out;
}

static vec3_t quaternion_rotate_inverse(vec3_t v, quat_t q) {
    quat_t q_inv = { q.w, -q.x, -q.y, -q.z };
    return quaternion_rotate(v, q_inv);
}

// ============================================================================
// State Space Integration Matrix Functions
// ============================================================================

static void rigid_body_derivative(const quad_state_t *s, const float current_thrusts[4], uint32_t time_us, quad_state_t *ds) {
    /* 1. First-Order Motor response lag */
    for (int i = 0; i < 4; i++) {
        ds->motor_thrust[i] = (current_thrusts[i] - s->motor_thrust[i]) / PLANT_TAU_MOTOR;
    }

    /* 2. Quad-X Mixing Allocation Equations */
    float sum_thrust = s->motor_thrust[0] + s->motor_thrust[1] + s->motor_thrust[2] + s->motor_thrust[3];
    
    float t_roll  = PLANT_ARM_LEN * (s->motor_thrust[0] - s->motor_thrust[1] - s->motor_thrust[2] + s->motor_thrust[3]);
    float t_pitch = PLANT_ARM_LEN * (s->motor_thrust[0] + s->motor_thrust[1] - s->motor_thrust[2] - s->motor_thrust[3]);
    float t_yaw   = PLANT_K_Q_OVER_KT * (s->motor_thrust[0] - s->motor_thrust[1] + s->motor_thrust[2] - s->motor_thrust[3]);

    /* 3. Newtonian Translation Kinematics */
    vec3_t f_body = { 0.0f, 0.0f, sum_thrust };
    vec3_t f_world = quaternion_rotate(f_body, s->orientation);
    
    ds->position = s->velocity;
    ds->velocity.x = f_world.x / PLANT_MASS_KG;
    ds->velocity.y = f_world.y / PLANT_MASS_KG;
    ds->velocity.z = (f_world.z / PLANT_MASS_KG) - PLANT_GRAVITY;

    /* 4. Euler's Rotational Rigid Frame Formulation with Transient Disturbance API */
    vec3_t total_torque = { t_roll, t_pitch, t_yaw };
    if (global_disturbance.duration_us > 0 &&
        time_us >= global_disturbance.start_us &&
        time_us < (global_disturbance.start_us + global_disturbance.duration_us)) {
        total_torque.x += global_disturbance.torque_Nm.x;
        total_torque.y += global_disturbance.torque_Nm.y;
        total_torque.z += global_disturbance.torque_Nm.z;
    }

    vec3_t I_omega = { PLANT_IXX * s->angular_rate.x, PLANT_IYY * s->angular_rate.y, PLANT_IZZ * s->angular_rate.z };
    vec3_t cross_gyroscopic = vector_cross(s->angular_rate, I_omega);

    ds->angular_rate.x = (total_torque.x - cross_gyroscopic.x) / PLANT_IXX;
    ds->angular_rate.y = (total_torque.y - cross_gyroscopic.y) / PLANT_IYY;
    ds->angular_rate.z = (total_torque.z - cross_gyroscopic.z) / PLANT_IZZ;

    /* 5. Quaternion Kinematics Execution Matrix */
    quat_t omega_q = { 0.0f, s->angular_rate.x, s->angular_rate.y, s->angular_rate.z };
    quat_t q_dot = quaternion_multiply(s->orientation, omega_q);
    ds->orientation.w = 0.5f * q_dot.w;
    ds->orientation.x = 0.5f * q_dot.x;
    ds->orientation.y = 0.5f * q_dot.y;
    ds->orientation.z = 0.5f * q_dot.z;
}

// ============================================================================
// Public Execution Implementation Boundaries
// ============================================================================

void plant_init(uint32_t seed, const quad_state_t *init_state) {
    global_seed = seed;
    memset(&global_disturbance, 0, sizeof(disturbance_t));

    if (init_state) {
        memcpy(&global_state, init_state, sizeof(quad_state_t));
    } else {
        memset(&global_state, 0, sizeof(quad_state_t));
        global_state.orientation.w = 1.0f;
        
        float base_eq = (PLANT_MASS_KG * PLANT_GRAVITY) / 4.0f;
        for (int i = 0; i < 4; i++) {
            global_state.motor_thrust[i] = base_eq;
        }
    }

    uint32_t internal_seed = global_seed + 7777;
    gyro_static_bias.x = generate_gaussian(&internal_seed, PLANT_SIGMA_BIAS_GYRO * (0.017453f)); /* Converts deg/s to rad/s bias explicitly */
    gyro_static_bias.y = generate_gaussian(&internal_seed, PLANT_SIGMA_BIAS_GYRO * (0.017453f));
    gyro_static_bias.z = generate_gaussian(&internal_seed, PLANT_SIGMA_BIAS_GYRO * (0.017453f));
}

void plant_inject_disturbance(const disturbance_t *d) {
    if (d) memcpy(&global_disturbance, d, sizeof(disturbance_t));
}

const quad_state_t *plant_get_state(void) {
    return &global_state;
}

void plant_step(const uint16_t throttle_cmd[4], uint32_t now_us) {
    float commanded_thrusts[4];
    for (int i = 0; i < 4; i++) {
        float u = (float)throttle_cmd[i];
        commanded_thrusts[i] = u * u * PLANT_K_T;
    }

    quad_state_t s = global_state;
    quad_state_t k1, k2, k3, k4;
    quad_state_t scratch;

    /* RK4 Solver Framework Sub-Ticks (Section 2.2 Numerical Tracking) */
    rigid_body_derivative(&s, commanded_thrusts, now_us, &k1);

    scratch.position.x = s.position.x + 0.5f * PLANT_DT_S * k1.position.x;
    scratch.position.y = s.position.y + 0.5f * PLANT_DT_S * k1.position.y;
    scratch.position.z = s.position.z + 0.5f * PLANT_DT_S * k1.position.z;
    scratch.velocity.x = s.velocity.x + 0.5f * PLANT_DT_S * k1.velocity.x;
    scratch.velocity.y = s.velocity.y + 0.5f * PLANT_DT_S * k1.velocity.y;
    scratch.velocity.z = s.velocity.z + 0.5f * PLANT_DT_S * k1.velocity.z;
    scratch.orientation.w = s.orientation.w + 0.5f * PLANT_DT_S * k1.orientation.w;
    scratch.orientation.x = s.orientation.x + 0.5f * PLANT_DT_S * k1.orientation.x;
    scratch.orientation.y = s.orientation.y + 0.5f * PLANT_DT_S * k1.orientation.y;
    scratch.orientation.z = s.orientation.z + 0.5f * PLANT_DT_S * k1.orientation.z;
    scratch.angular_rate.x = s.angular_rate.x + 0.5f * PLANT_DT_S * k1.angular_rate.x;
    scratch.angular_rate.y = s.angular_rate.y + 0.5f * PLANT_DT_S * k1.angular_rate.y;
    scratch.angular_rate.z = s.angular_rate.z + 0.5f * PLANT_DT_S * k1.angular_rate.z;
    for(int i=0; i<4; i++) scratch.motor_thrust[i] = s.motor_thrust[i] + 0.5f * PLANT_DT_S * k1.motor_thrust[i];
    rigid_body_derivative(&scratch, commanded_thrusts, now_us, &k2);

    scratch.position.x = s.position.x + 0.5f * PLANT_DT_S * k2.position.x;
    scratch.position.y = s.position.y + 0.5f * PLANT_DT_S * k2.position.y;
    scratch.position.z = s.position.z + 0.5f * PLANT_DT_S * k2.position.z;
    scratch.velocity.x = s.velocity.x + 0.5f * PLANT_DT_S * k2.velocity.x;
    scratch.velocity.y = s.velocity.y + 0.5f * PLANT_DT_S * k2.velocity.y;
    scratch.velocity.z = s.velocity.z + 0.5f * PLANT_DT_S * k2.velocity.z;
    scratch.orientation.w = s.orientation.w + 0.5f * PLANT_DT_S * k2.orientation.w;
    scratch.orientation.x = s.orientation.x + 0.5f * PLANT_DT_S * k2.orientation.x;
    scratch.orientation.y = s.orientation.y + 0.5f * PLANT_DT_S * k2.orientation.y;
    scratch.orientation.z = s.orientation.z + 0.5f * PLANT_DT_S * k2.orientation.z;
    scratch.angular_rate.x = s.angular_rate.x + 0.5f * PLANT_DT_S * k2.angular_rate.x;
    scratch.angular_rate.y = s.angular_rate.y + 0.5f * PLANT_DT_S * k2.angular_rate.y;
    scratch.angular_rate.z = s.angular_rate.z + 0.5f * PLANT_DT_S * k2.angular_rate.z;
    for(int i=0; i<4; i++) scratch.motor_thrust[i] = s.motor_thrust[i] + 0.5f * PLANT_DT_S * k2.motor_thrust[i];
    rigid_body_derivative(&scratch, commanded_thrusts, now_us, &k3);

    scratch.position.x = s.position.x + PLANT_DT_S * k3.position.x;
    scratch.position.y = s.position.y + PLANT_DT_S * k3.position.y;
    scratch.position.z = s.position.z + PLANT_DT_S * k3.position.z;
    scratch.velocity.x = s.velocity.x + PLANT_DT_S * k3.velocity.x;
    scratch.velocity.y = s.velocity.y + PLANT_DT_S * k3.velocity.y;
    scratch.velocity.z = s.velocity.z + PLANT_DT_S * k3.velocity.z;
    scratch.orientation.w = s.orientation.w + PLANT_DT_S * k3.orientation.w;
    scratch.orientation.x = s.orientation.x + PLANT_DT_S * k3.orientation.x;
    scratch.orientation.y = s.orientation.y + PLANT_DT_S * k3.orientation.y;
    scratch.orientation.z = s.orientation.z + PLANT_DT_S * k3.orientation.z;
    scratch.angular_rate.x = s.angular_rate.x + PLANT_DT_S * k3.angular_rate.x;
    scratch.angular_rate.y = s.angular_rate.y + PLANT_DT_S * k3.angular_rate.y;
    scratch.angular_rate.z = s.angular_rate.z + PLANT_DT_S * k3.angular_rate.z;
    for(int i=0; i<4; i++) scratch.motor_thrust[i] = s.motor_thrust[i] + PLANT_DT_S * k3.motor_thrust[i];
    rigid_body_derivative(&scratch, commanded_thrusts, now_us, &k4);

    /* Average Accumulator */
    global_state.position.x += (PLANT_DT_S / 6.0f) * (k1.position.x + 2.0f * k2.position.x + 2.0f * k3.position.x + k4.position.x);
    global_state.position.y += (PLANT_DT_S / 6.0f) * (k1.position.y + 2.0f * k2.position.y + 2.0f * k3.position.y + k4.position.y);
    global_state.position.z += (PLANT_DT_S / 6.0f) * (k1.position.z + 2.0f * k2.position.z + 2.0f * k3.position.z + k4.position.z);

    global_state.velocity.x += (PLANT_DT_S / 6.0f) * (k1.velocity.x + 2.0f * k2.velocity.x + 2.0f * k3.velocity.x + k4.velocity.x);
    global_state.velocity.y += (PLANT_DT_S / 6.0f) * (k1.velocity.y + 2.0f * k2.velocity.y + 2.0f * k3.velocity.y + k4.velocity.y);
    global_state.velocity.z += (PLANT_DT_S / 6.0f) * (k1.velocity.z + 2.0f * k2.velocity.z + 2.0f * k3.velocity.z + k4.velocity.z);

    global_state.orientation.w += (PLANT_DT_S / 6.0f) * (k1.orientation.w + 2.0f * k2.orientation.w + 2.0f * k3.orientation.w + k4.orientation.w);
    global_state.orientation.x += (PLANT_DT_S / 6.0f) * (k1.orientation.x + 2.0f * k2.orientation.x + 2.0f * k3.orientation.x + k4.orientation.x);
    global_state.orientation.y += (PLANT_DT_S / 6.0f) * (k1.orientation.y + 2.0f * k2.orientation.y + 2.0f * k3.orientation.y + k4.orientation.y);
    global_state.orientation.z += (PLANT_DT_S / 6.0f) * (k1.orientation.z + 2.0f * k2.orientation.z + 2.0f * k3.orientation.z + k4.orientation.z);

    global_state.angular_rate.x += (PLANT_DT_S / 6.0f) * (k1.angular_rate.x + 2.0f * k2.angular_rate.x + 2.0f * k3.angular_rate.x + k4.angular_rate.x);
    global_state.angular_rate.y += (PLANT_DT_S / 6.0f) * (k1.angular_rate.y + 2.0f * k2.angular_rate.y + 2.0f * k3.angular_rate.y + k4.angular_rate.y);
    global_state.angular_rate.z += (PLANT_DT_S / 6.0f) * (k1.angular_rate.z + 2.0f * k2.angular_rate.z + 2.0f * k3.angular_rate.z + k4.angular_rate.z);

    for (int i = 0; i < 4; i++) {
        global_state.motor_thrust[i] += (PLANT_DT_S / 6.0f) * (k1.motor_thrust[i] + 2.0f * k2.motor_thrust[i] + 2.0f * k3.motor_thrust[i] + k4.motor_thrust[i]);
    }

    /* Enforce Quat Normalization (Section 6.1 Constraint) */
    float norm = sqrtf(global_state.orientation.w * global_state.orientation.w + 
                       global_state.orientation.x * global_state.orientation.x + 
                       global_state.orientation.y * global_state.orientation.y + 
                       global_state.orientation.z * global_state.orientation.z);
    if (norm > 0.0f) {
        global_state.orientation.w /= norm; global_state.orientation.x /= norm;
        global_state.orientation.y /= norm; global_state.orientation.z /= norm;
    } else {
        global_state.orientation.w = 1.0f; global_state.orientation.x = 0.0f;
        global_state.orientation.y = 0.0f; global_state.orientation.z = 0.0f;
    }
}

// ============================================================================
// Clean Array Exporters For Drivers Mapping 1-to-1
// ============================================================================

void plant_generate_gyro(float out_gyro[3], uint32_t ts_us) {
    uint32_t local_seed = global_seed + ts_us + 11;
    out_gyro[0] = global_state.angular_rate.x + gyro_static_bias.x + generate_gaussian(&local_seed, PLANT_SIGMA_GYRO);
    out_gyro[1] = global_state.angular_rate.y + gyro_static_bias.y + generate_gaussian(&local_seed, PLANT_SIGMA_GYRO);
    out_gyro[2] = global_state.angular_rate.z + gyro_static_bias.z + generate_gaussian(&local_seed, PLANT_SIGMA_GYRO);
}

void plant_generate_accel(float out_accel[3], uint32_t ts_us) {
    uint32_t local_seed = global_seed + ts_us + 22;
    
    float total_thrust = global_state.motor_thrust[0] + global_state.motor_thrust[1] + global_state.motor_thrust[2] + global_state.motor_thrust[3];
    vec3_t dynamic_accel_body = { 0.0f, 0.0f, total_thrust / PLANT_MASS_KG };
    
    vec3_t gravity_world = { 0.0f, 0.0f, -PLANT_GRAVITY };
    vec3_t gravity_body = quaternion_rotate_inverse(gravity_world, global_state.orientation);

    out_accel[0] = dynamic_accel_body.x - gravity_body.x + generate_gaussian(&local_seed, PLANT_SIGMA_ACCEL);
    out_accel[1] = dynamic_accel_body.y - gravity_body.y + generate_gaussian(&local_seed, PLANT_SIGMA_ACCEL);
    out_accel[2] = dynamic_accel_body.z - gravity_body.z + generate_gaussian(&local_seed, PLANT_SIGMA_ACCEL);
}

void plant_generate_mag(float out_mag[3], uint32_t ts_us) {
    uint32_t local_seed = global_seed + ts_us + 33;
    
    vec3_t earth_field_world = { PLANT_EARTH_FIELD_X, PLANT_EARTH_FIELD_Y, PLANT_EARTH_FIELD_Z };
    vec3_t mag_body_clean = quaternion_rotate_inverse(earth_field_world, global_state.orientation);

    out_mag[0] = mag_body_clean.x + PLANT_HARD_IRON_X + generate_gaussian(&local_seed, PLANT_SIGMA_MAG);
    out_mag[1] = mag_body_clean.y + PLANT_HARD_IRON_Y + generate_gaussian(&local_seed, PLANT_SIGMA_MAG);
    out_mag[2] = mag_body_clean.z + PLANT_HARD_IRON_Z + generate_gaussian(&local_seed, PLANT_SIGMA_MAG);
}