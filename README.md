# Flight Controller Simulator

A bare-metal flight controller implementation in C with physics simulation for testing.

## Architecture

- **Control Core:** Cascade PID for attitude control, Mahony AHRS for attitude estimation
- **HAL Layer:** Hardware abstraction for motors, sensors, and timing
- **Simulation:** Physics model with quadcopter dynamics
- **Testing:** Multiple scenarios for validation

## Build

Windows (MinGW):
```cmd
rmdir /s /q build && mkdir build && cd build
cmake -G "MinGW Makefiles" -D CMAKE_C_COMPILER=c:/mingw/bin/gcc.exe ..
cmake --build .
cd ..
```

## Run Tests

```cmd
build\sim_binary.exe --scenario hover
build\sim_binary.exe --scenario tilt_recovery
build\sim_binary.exe --scenario disturbance
```

## View Results

Python plotting requires pandas and matplotlib:
```cmd
python scripts\plot_scenario.py logs\hover.csv
python scripts\plot_scenario.py logs\tilt_recovery.csv
python scripts\plot_scenario.py logs\disturbance.csv
```

## Features

- **Attitude Estimation:** Quaternion-based Mahony filter
- **Control:** Cascade PID with separate angle and rate loops
- **Motor Control:** DShot protocol encoding
- **Altitude Hold:** Optional PID altitude controller
- **Simulation:** Deterministic 1000Hz execution loop
- **Logging:** 28-parameter telemetry at 100Hz

## Code Structure

```
src/               - Flight controller source
├── fc_main.c      - Main loop
├── attitude.c     - AHRS/attitude estimation
├── control.c      - PID controllers
├── mixer.c        - Motor mixing
├── dshot.c        - Motor protocol
└── modes.c        - Flight modes

sim/               - Physics simulation
├── plant.c        - Quadcopter dynamics
├── scenarios.c    - Test scenarios
└── sim_main.c     - Test runner

hal/               - Hardware abstraction
drivers_host/      - Mock drivers for simulation
```
