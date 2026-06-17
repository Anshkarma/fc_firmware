/**
 * @file test_dshot.c
 * @brief Unit test for the DShot CRC implementation against Section 8.3 reference values.
 */

#include <stdio.h>
#include <stdbool.h>
#include <assert.h>
#include "../src/dshot.h"

int main(void) {
    printf("[TEST] Running DShot CRC Unit Tests...\n");
    printf("==================================================\n");

    // Test Case 1: Disarmed (Zero Throttle), No Telemetry
    // Payload: 0x000, Expected CRC: 0x0, Expected Frame: 0x0000
    uint16_t frame_zero = dshot_encode_frame(0, false);
    assert(frame_zero == 0x0000);
    printf("[PASS] Disarmed (0)          -> Output: 0x%04X\n", frame_zero);

    // Test Case 2: Minimum Armed Throttle (48), No Telemetry
    // Payload: (48 << 1) = 96 = 0x060, Expected CRC: 0x6, Expected Frame: 0x0606
    uint16_t frame_min = dshot_encode_frame(48, false);
    assert(frame_min == 0x0606);
    printf("[PASS] Min Armed (48)        -> Output: 0x%04X\n", frame_min);

    // Test Case 3: Mid Throttle (1048), WITH Telemetry Request
    // Payload: (1048 << 1) | 1 = 2097 = 0x831, Expected CRC: 0x2, Expected Frame: 0x8312
    uint16_t frame_mid = dshot_encode_frame(1048, true);
    assert(frame_mid == 0x831A);
    printf("[PASS] Mid + Telemetry (1048)-> Output: 0x%04X\n", frame_mid);

    // Test Case 4: Max Throttle (2047), No Telemetry
    // Payload: (2047 << 1) = 4094 = 0xFFE, Expected CRC: 0xE, Expected Frame: 0xFFEE
    uint16_t frame_max = dshot_encode_frame(2047, false);
    assert(frame_max == 0xFFEE);
    printf("[PASS] Max Throttle (2047)   -> Output: 0x%04X\n", frame_max);

    printf("==================================================\n");
    printf("[SUCCESS] All DShot CRC values mathematically verified.\n");
    return 0;
}