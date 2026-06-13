#include "motor_hal.h"

/* Global array to hold the latest motor commands so sim_main.c can feed them to the plant */
uint16_t mock_motor_commands[4] = {0, 0, 0, 0};
static bool is_armed = false;

static bool mock_motor_init(void) {
    is_armed = false;
    mock_motor_commands[0] = 0;
    mock_motor_commands[1] = 0;
    mock_motor_commands[2] = 0;
    mock_motor_commands[3] = 0;
    return true;
}

static bool mock_send_dshot(const uint16_t motors[4]) {
    if (!motors) return false;
    
    /* Hardware Safety Guard: Only pass power to motors if the system is actually armed */
    if (is_armed) {
        mock_motor_commands[0] = motors[0];
        mock_motor_commands[1] = motors[1];
        mock_motor_commands[2] = motors[2];
        mock_motor_commands[3] = motors[3];
    } else {
        mock_motor_commands[0] = 0;
        mock_motor_commands[1] = 0;
        mock_motor_commands[2] = 0;
        mock_motor_commands[3] = 0;
    }
    return true;
}

static bool mock_arm(void) {
    is_armed = true;
    return true;
}

static bool mock_disarm(void) {
    is_armed = false;
    mock_motor_commands[0] = 0;
    mock_motor_commands[1] = 0;
    mock_motor_commands[2] = 0;
    mock_motor_commands[3] = 0;
    return true;
}

/* Concrete interface structure instance registration */
motor_dev_t motor_mock = {
    .init = mock_motor_init,
    .send_dshot = mock_send_dshot,
    .arm = mock_arm,
    .disarm = mock_disarm
};