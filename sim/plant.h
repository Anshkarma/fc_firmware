#ifndef PLANT_H
#define PLANT_H

#include <stdint.h>
#include <stdbool.h>

/* Physical Parameters Configuration Core (Section 6.1) */
#define PLANT_MASS_KG         0.250f       /* ~250g Class Drone Weight */
#define PLANT_IXX             2.3e-3f      /* Moment of Inertia roll axis, kg*m^2 */
#define PLANT_IYY             2.3e-3f      /* Moment of Inertia pitch axis, kg*m^2 */
#define PLANT_IZZ             4.6e-3f      /* Moment of Inertia yaw axis, kg*m^2 */
#define PLANT_ARM_LEN         0.1f         /* Center to motor distance, meters */
#define PLANT_GRAVITY         9.81f        /* Constant acceleration, m/s^2 */
#define PLANT_TAU_MOTOR       0.030f       /* Motor first-order lag time, seconds */
#define PLANT_K_T             1.0e-5f      /* Thrust coefficient factor */
#define PLANT_K_Q_OVER_KT     0.02f        /* Dimensionless yaw/thrust scaling ratio */

/* Stochastic Noise Standard Deviations Matrix (Section 6.2) */
#define PLANT_SIGMA_GYRO      0.05f        /* Gyro wideband noise, rad/s */
#define PLANT_SIGMA_BIAS_GYRO 1.0f         /* Gyro random walk bias initialization index */
#define PLANT_SIGMA_ACCEL     0.05f        /* Accelerometer noise spectrum, m/s^2 */
#define PLANT_SIGMA_MAG       0.05f        /* Magnetometer noise deviation scale, uT */

/* Local Earth Field Settings (NED Vector) */
#define PLANT_EARTH_FIELD_X   20.0f        /* North Component, uT */
#define PLANT_EARTH_FIELD_Y   0.0f         /* East Component, uT */
#define PLANT_EARTH_FIELD_Z   -44.0f       /* Down Component, uT */

/* Distortion Offset Vectors */
#define PLANT_HARD_IRON_X     5.0f         /* Hard-iron magnetic bias X, uT */
#define PLANT_HARD_IRON_Y     -3.0f        /* Hard-iron magnetic bias Y, uT */
#define PLANT_HARD_IRON_Z     2.0f         /* Hard-iron magnetic bias Z, uT */

/* Timing Cycle Frames */
#define PLANT_DT_US           1000u        /* 1 kHz baseline tick internal, us */
#define PLANT_DT_S            (PLANT_DT_US * 1.0e-6f)

/* Vector & Hamilton Scalar-First Quaternion Representations */
typedef struct { float x; float y; float z; } vec3_t;
typedef struct { float w; float x; float y; float z; } quat_t;

/**
 * @brief Full physical environment tracking matrix (17 float state)
 */
typedef struct {
    vec3_t  position;          /* Earth frame coordinates, m */
    vec3_t  velocity;          /* Earth frame velocities, m/s */
    quat_t  orientation;       /* Body-to-World transformation orientation state */
    vec3_t  angular_rate;      /* Body frame angular rates, rad/s */
    float   motor_thrust[4];   /* Dynamic actual motor thrust force, N */
} quad_state_t;

typedef struct {
    vec3_t   torque_Nm;        /* Impulse torque vector payload */
    uint32_t start_us;         /* Trigger timeline step marker, us */
    uint32_t duration_us;      /* Action window width, us */
} disturbance_t;

/* Continuous Simulation Environment Core Interfaces */
void plant_init(uint32_t seed, const quad_state_t *init_state);
void plant_step(const uint16_t throttle_cmd[4], uint32_t now_us);
const quad_state_t *plant_get_state(void);
void plant_inject_disturbance(const disturbance_t *d);

/* Platform-Agnostic Raw Generators Passed Into Mock Driver Bindings */
void plant_generate_gyro(float out_gyro[3], uint32_t ts_us);
void plant_generate_accel(float out_accel[3], uint32_t ts_us);
void plant_generate_mag(float out_mag[3], uint32_t ts_us);

#endif /* PLANT_H */