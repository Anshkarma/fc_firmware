cmake_minimum_required(VERSION 3.15)
project(flight_controller_sim C)

# Force strict embedded compilation constraints
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)

if(MSVC)
    add_compile_options(/W4 /WX)
else()
    add_compile_options(-Wall -Wextra -Wpedantic -O2)
endif()

# Define Source Collections
set(HAL_MOCK_SOURCES
    drivers_host/imu_mock.c
    drivers_host/mag_mock.c
    drivers_host/motor_mock.c
    drivers_host/time_mock.c
    drivers_host/baro_mock.c
)

set(FIRMWARE_CORE_SOURCES
    src/config.c
    src/attitude.c
    src/altitude.c
    src/control.c
    src/mixer.c
    src/dshot.c
    src/modes.c
    src/fc_main.c
)

set(SIMULATION_SOURCES
    sim/plant.c
    sim/scenarios.c
    sim/sim_main.c
    sim/visualizer.c
)

# 1. Target: The Main Simulation Engine (sim_binary)
add_executable(sim_binary 
    ${HAL_MOCK_SOURCES}
    ${FIRMWARE_CORE_SOURCES}
    ${SIMULATION_SOURCES}
)

# Setup Includes
target_include_directories(sim_binary PRIVATE 
    ${CMAKE_CURRENT_SOURCE_DIR}/hal
    ${CMAKE_CURRENT_SOURCE_DIR}/src
    ${CMAKE_CURRENT_SOURCE_DIR}/sim
    ${CMAKE_CURRENT_SOURCE_DIR}/drivers_host
    ${CMAKE_CURRENT_SOURCE_DIR}/raylib/include
)

# Link Libraries for Main Target
target_link_directories(sim_binary PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/raylib/lib)
target_link_libraries(sim_binary PRIVATE raylib winmm gdi32 opengl32 m)

# 2. Target: The Isolated DShot Unit Test
add_executable(test_dshot 
    src/dshot.c
    tests/test_dshot_crc.c
)

# Link math library for test target
if(NOT MSVC)
    target_link_libraries(test_dshot PRIVATE m)
endif()