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

set(FIRMWARE_CORE_SOURCES
    src/attitude.c
    src/control.c
    src/mixer.c
    src/dshot.c
)

set(SIMULATION_SOURCES
    sim/plant.c
    sim/sim_main.c
)

# Declare target binary executable matrix
add_executable(sim_binary 
    ${HAL_MOCK_SOURCES}
    ${FIRMWARE_CORE_SOURCES}
    ${SIMULATION_SOURCES}
)

# Link standard math primitives framework cleanly
if(NOT MSVC)
    target_link_libraries(sim_binary m)
endif()