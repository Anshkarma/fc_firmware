@echo off

echo [1] Obliterating old cache and logs...
if exist build rmdir /s /q build

if not exist old_logs mkdir old_logs
if exist logs\*.png move /Y logs\*.png old_logs\

del /q logs\*.csv

echo [2] Compiling the Firmware and Plant...
mkdir build
cd build
cmake -G "MinGW Makefiles" -D CMAKE_C_COMPILER=c:/mingw/bin/gcc.exe ..
cmake --build .
cd ..

echo [3] Executing Simulation Scenarios...
build\sim_binary.exe --scenario hover
build\sim_binary.exe --scenario tilt_recovery 
build\sim_binary.exe --scenario disturbance

echo [4] Generating Telemetry Plots...
python scripts\plot_all.py

@echo on