/**
 * @file dshot.c
 * @brief DShot encoder implementation (MSB First, 11-bit throttle + 1-bit Telemetry + 4-bit CRC).
 */

#include "dshot.h"
#include <math.h>

// Global protocol state (affects driver timing, not the bit encoding)
static dshot_protocol_t current_protocol = DSHOT_PROTOCOL_600;

void dshot_set_protocol(dshot_protocol_t protocol) {
    current_protocol = protocol;
}

dshot_protocol_t dshot_get_protocol(void) {
    return current_protocol;
}

/**
 * @brief Low-level function: Takes wire-format throttle and produces a 16-bit frame with CRC.
 * @param throttle_2047 11-bit wire format throttle (0-2047).
 * @param telemetry True to request telemetry from the ESC.
 * @return 16-bit encoded DShot frame.
 */
uint16_t dshot_encode_frame(uint16_t throttle_2047, bool telemetry) {
    // Pack 11-bit throttle and 1-bit telemetry into 12 bits
    uint16_t data = (throttle_2047 << 1) | (telemetry ? 1 : 0);
    
    // Calculate 4-bit CRC over the 12 bits using XOR folding (Section 8.2)
    uint8_t crc = (data ^ (data >> 4) ^ (data >> 8)) & 0x0F;
    
    // Append CRC to the lower 4 bits
    return (data << 4) | crc;
}

/**
 * @brief High-level function: Normalizes 0.0-1.0 float to wire format and encodes.
 * @param throttle_norm Normalized throttle command [0.0, 1.0].
 * @param telemetry True to request telemetry.
 * @param armed System arming state (disarmed sends strict 0).
 * @return 16-bit encoded DShot frame.
 */
uint16_t dshot_encode(float throttle_norm, bool telemetry, bool armed) {
    // Safety check: Disarmed overrides everything to zero
    if (!armed) {
        return dshot_encode_frame(0, 0); 
    }

    // Clamp normalized throttle strictly between 0.0 and 1.0
    float t = throttle_norm;
    if (t < 0.0f) t = 0.0f;
    if (t > 1.0f) t = 1.0f;

    // Linear map to DShot armed range: 48 to 2047
    uint16_t throttle_2047 = 48 + (uint16_t)roundf(t * (2047.0f - 48.0f));

    return dshot_encode_frame(throttle_2047, telemetry);
}

/**
 * @brief Public API: Processes all 4 motor commands simultaneously.
 * @param throttle_norm Array of 4 normalized throttle values.
 * @param telemetry Telemetry flag (applies to all motors).
 * @param armed System arming state.
 * @param frames Array populated with 4 encoded 16-bit frames.
 */
void dshot_encode_motors(const float throttle_norm[4], bool telemetry, bool armed, uint16_t frames[4]) {
    for (int i = 0; i < 4; i++) {
        frames[i] = dshot_encode(throttle_norm[i], telemetry, armed);
    }
}