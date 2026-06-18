# Deterministic Flight Controller Simulation Bench

## Architecture Overview
This repository contains a bare-metal, strictly deterministic flight controller architecture written in pure C. The system is designed with a relentless focus on the **Separation of Concerns**. 

The core flight dynamics algorithms—including a Cascade PID controller and a mathematically rigorous Mahony AHRS (Attitude and Heading Reference System)—are 100% hardware-agnostic. They interface with the physical world (or the integrated physics plant) exclusively through isolated Hardware Abstraction Layer (HAL) mock drivers (`motor_mock` and `time_mock`).

## 🛠️ 1. Build Instructions
The build pipeline is optimized for a minimalist command-line environment utilizing CMake and MinGW GCC. 

Open your Command Prompt at the project root and execute the following chain to purge legacy artifacts and force a clean, definitive build:

```cmd
rmdir /s /q build && mkdir build && cd build && cmake -G "MinGW Makefiles" -D CMAKE_C_COMPILER=c:/mingw/bin/gcc.exe .. && cmake --build . && cd ..

cmake --build .

cd ..
```


2. Execution Instructions
The central orchestration engine (sim_main.c) is designed to evaluate the firmware against strict mathematical test scenarios. The engine operates on a synthetic 1000Hz execution loop.

Run the compiled binary directly from your terminal, passing the target scenario as an argument:
```cmd
build\sim_main.exe --scenario hover

build\sim_main.exe --scenario tilt_recovery

build\sim_main.exe --scenario disturbance
``` 

3. Telemetry & Plotting
To facilitate brutal empirical scrutiny, the simulation downsamples the 1000Hz internal physics tick rate to a 100Hz telemetry log.

Locate Data: Upon successful scenario completion, the engine automatically flushes the pipeline to logs/<scenario_name>.csv.

Data Structure: The CSV strictly adheres to a 28-column matrix, capturing everything from the plant's true rigid-body state to the raw 16-bit encoded DShot frames dispatched by the HAL.

Visualization: Feed the generated artifact into your Python plotting utility to extract visual verification of the system's performance.
```
DOS

python script\plot_all.py
```
or you can do it scenriowise
```
python scripts\plot_scenario.py logs\tilt_recovery.csv

python scripts\plot_scenario.py logs\disturbance.csv

python scripts\plot_scenario.py logs\hover.csv
```
both the .py files have '--help' feature
