/**
 * @file types.h
 * @brief Global type definitions for the flight controller firmware.
 */

#ifndef TYPES_H
#define TYPES_H

typedef struct {
    float x;
    float y;
    float z;
} vec3_t;

typedef struct {
    float w;
    float x;
    float y;
    float z;
} quat_t;

typedef struct {
    float roll;
    float pitch;
    float yaw;
} control_torque_t;

typedef struct {
float z;      // Estimated altitude (meters)
float v;      // Estimated vertical velocity (m/s)

// Error Covariance Matrix (2x2)
float P[2][2];

// Process Noise Covariance (Tuning Parameters)
float Q_pos;
float Q_vel;

// Measurement Noise Covariance
float R_baro;


} kalman_alt_t;



#endif // TYPES_H