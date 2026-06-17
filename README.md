\# Isolated Flight Controller \& Simulation Testbench Framework



This repository contains an isolated, production-grade Flight Controller firmware integrated with a deterministic 4-axis rigid body simulation plant.



\## 🛠️ Build and Compilation Instructions



The firmware utilizes CMake to generate host-compliant binaries with maximum warning severity levels to ensure compliance.



```cmd

:: Create and navigate to the build environment

mkdir build

cd build



:: Generate MinGW compilation makefiles via GCC

cmake -G "MinGW Makefiles" -D CMAKE\_C\_COMPILER=c:/mingw/bin/gcc.exe ..



:: Compile all targets (Firmware Binary, Simulation Testbench, and Unit Tests)

cmake --build .

cd ..

