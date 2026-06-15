/**
 * @file dshot.h
 * @brief DShot protocol encoder for 16-bit wire-format frames.
 * * This module is completely isolated and portable. It converts normalized
 * thrust commands into bit-banged or DMA-ready 16-bit DShot frames.
 */

#ifndef DSHOT_H
#define DSHOT_H

#include <stdint.h>
#include <stdbool.h>

/**
 * @brief Configurable protocol speeds.
 * Note: Frame bit-structure remains identical; this simply acts as a 
 * logged tag for target hardware timers to read.
 */
typedef enum {
    DSHOT_PROTOCOL_300,
    DSHOT_PROTOCOL_600
} dshot_protocol_t;

// Configuration API
void dshot_set_protocol(dshot_protocol_t protocol);
dshot_protocol_t dshot_get_protocol(void);

// Low-Level API (Section 8.2)
uint16_t dshot_encode_frame(uint16_t throttle_2047, bool telemetry);

// High-Level Conversion APIs
uint16_t dshot_encode(float throttle_norm, bool telemetry, bool armed);
void dshot_encode_motors(const float throttle_norm[4], bool telemetry, bool armed, uint16_t frames[4]);

#endif // DSHOT_H