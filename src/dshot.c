/**
 * @file dshot.c
 * @brief DShot protocol encoder for translating normalized motor commands 
 * into 16-bit digital ESC frames.
 */

#include "dshot.h"
#include <math.h>

// ==============================================================================
// INTERNAL CORE ALGORITHMS
// ==============================================================================

/**
 * @brief Generates the 16-bit DShot frame including the 4-bit CRC.
 * Strictly adheres to standard DShot XOR folding (Section 8.2).
 */
uint16_t dshot_encode_frame(uint16_t throttle_2047, bool telemetry) {
    // 1. Pack 11-bit throttle and 1-bit telemetry into 12 bits
    uint16_t data = (throttle_2047 << 1) | (telemetry ? 1 : 0);
    
    // 2. Calculate 4-bit CRC over the 12 bits using standard XOR folding
    uint16_t crc = (data ^ (data >> 4) ^ (data >> 8)) & 0x0F;
    
    // 3. Assemble and return the 16-bit payload
    return (data << 4) | crc;
}

// ==============================================================================
// PUBLIC APIs
// ==============================================================================

/**
 * @brief Encodes a single normalized float throttle into a DShot frame.
 */
uint16_t dshot_encode(float throttle_norm, bool telemetry, bool armed) {
    // Safety check: Disarmed explicitly forces the kill signal (0)
    if (!armed) {
        return dshot_encode_frame(0, false); 
    }

    // Clamp normalized throttle strictly to prevent scaling overflow
    float t = throttle_norm;
    if (t < 0.0f) t = 0.0f;
    if (t > 1.0f) t = 1.0f;

    // Linear mapping to the strict DShot armed range (48 to 2047)
    uint16_t throttle_2047 = 48 + (uint16_t)roundf(t * (2047.0f - 48.0f));

    return dshot_encode_frame(throttle_2047, telemetry);
}

/**
 * @brief Encodes the full quadcopter motor array into 16-bit frames.
 */
void dshot_encode_motors(const float motor_norm[4], bool telemetry, bool is_armed, uint16_t output_frames[4]) {
    for (int i = 0; i < 4; i++) {
        // Dispatch mapping sequence to the atomic encoder
        output_frames[i] = dshot_encode(motor_norm[i], telemetry, is_armed);
    }
}


void dshot_set_protocol(dshot_protocol_t protocol) {
    // Hardware configuration logic goes here for real flight controller
    // (void)protocol; prevents compiler warning for unused variable in simulation
    (void)protocol; 
}