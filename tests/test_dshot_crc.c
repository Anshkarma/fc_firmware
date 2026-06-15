/**
 * @file test_dshot_crc.c
 * @brief Unit tests for DShot Frame Encoder against Section 8.3 specification.
 */

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

// Directly include the implementation for unit testing isolation
#include "../src/dshot.h" 

typedef struct {
    uint16_t throttle;
    bool telemetry;
    uint16_t expected_frame;
    const char* notes;
} test_case_t;

int main(void) {
    // Reference values from Section 8.3
    test_case_t tests[] = {
        {0,    false, 0x0000, "Disarmed"},
        {48,   false, 0x0606, "Minimum armed throttle"},
        {1024, false, 0x807F, "Mid throttle"},
        {2047, false, 0xFFE0, "Max throttle"}
    };

    int num_tests = sizeof(tests) / sizeof(tests[0]);
    int passed = 0;

    printf("[TEST] Booting DShot CRC Encoder Validation Engine...\n");
    printf("=========================================================\n");

    for (int i = 0; i < num_tests; i++) {
        uint16_t result = dshot_encode_frame(tests[i].throttle, tests[i].telemetry);
        
        if (result == tests[i].expected_frame) {
            printf("[PASS] %s \n       Input: %u -> Output: 0x%04X (Expected: 0x%04X)\n", 
                   tests[i].notes, tests[i].throttle, result, tests[i].expected_frame);
            passed++;
        } else {
            printf("[FAIL] %s \n       Input: %u -> Output: 0x%04X (Expected: 0x%04X)\n", 
                   tests[i].notes, tests[i].throttle, result, tests[i].expected_frame);
        }
    }

    printf("=========================================================\n");
    if (passed == num_tests) {
        printf("[SUCCESS] All %d tests passed! DShot encoding aligns with Section 8.3.\n", num_tests);
        return 0; // Standard exit success
    } else {
        printf("[ERROR] %d out of %d tests failed. Firmware compilation should abort.\n", 
               num_tests - passed, num_tests);
        return -1; // Standard exit failure
    }
}