#include "time_hal.h"

/* Global simulation clock tracked in microseconds. 
   Driven and advanced directly by sim_main.c */
uint32_t current_sim_time_us = 0;

static uint32_t mock_micros(void) {
    return current_sim_time_us;
}

static uint32_t mock_millis(void) {
    return current_sim_time_us / 1000;
}

/* Concrete interface structure instance registration */
time_dev_t time_mock = {
    .micros = mock_micros,
    .millis = mock_millis
};