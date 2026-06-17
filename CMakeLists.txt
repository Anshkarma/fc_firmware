cmake_minimum_required(VERSION 3.15)
project(flight_controller_sim C)

# Force strict embedded compilation constraints (§1.2 Rule Compliance)
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)

if(MSVC)
    add_compile_options(/W4 /WX)
else()
    add_compile_options(-Wall -Wextra -Wpedantic -O2)
endif()

# Explicitly discover all system directories
include_directories(hal)
include_directories(src)
include_directories(sim)

# Accumulate separate source targets to preserve translation unit boundaries
set(HAL_MOCK_SOURCES
    drivers_host/imu_mock.c
    drivers_host/mag_mock.c
    drivers_host/motor_mock.c
    drivers_host/time_mock.c
)

# Added the newly engineered modules into the compilation chain
set(FIRMWARE_CORE_SOURCES
    src/config.c      # Global PID gains and tuning params
    src/attitude.c
    src/control.c
    src/mixer.c
    src/dshot.c
    src/modes.c       # Flight mode state machine
    src/fc_main.c     # The 1000Hz Periodic Orchestrator
)

set(SIMULATION_SOURCES
    sim/plant.c
    sim/scenarios.c
    sim/sim_main.c
)

# 1. Target: The Main Simulation Engine
add_executable(sim_binary 
    ${HAL_MOCK_SOURCES}
    ${FIRMWARE_CORE_SOURCES}
    ${SIMULATION_SOURCES}
)

# 2. Target: The Isolated DShot Unit Test (Section 8.3 Verification)
add_executable(test_dshot 
    src/dshot.c
    tests/test_dshot_crc.c
)

# Link standard math primitives framework cleanly
if(NOT MSVC)
    target_link_libraries(sim_binary m)
    target_link_libraries(test_dshot m)
endif()