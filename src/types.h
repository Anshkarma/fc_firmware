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

#endif // TYPES_H