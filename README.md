# AirSim-Onboard
Microsoft AirSim multi-rotor implementation for DJI Onboard-SDK compatible flight controllers

Project code-name: Rosie

_Work in progress_

Milestone: v0.3

## Details 
* New Onboard-SDK compatible AirSim multi-rotor API implementation (OnboardLib/.../OnboardMultirotorApi.hpp)
* New Onboard-SDK compatible AirSim DroneServer implementation (OnboardServer/main.cpp)
* Linux Only - DJI Onboard SDK not available for Windows
* Requires GNU C++ Compiler (Tested with G++-6 and G++-7)
* Tested with standard DroneShell RPC client. Should work with other clients
* Works with HIL simulation (using DJI Simulator) or real flight mode
* No Unreal or Unity implementation (yet)

## TODO
* Video support
* Unreal/Unity simulation support
* Instructions for how to compile
* Refactor multi-rotor API implementation
* Investigate issues with LLVM compiler
* Fix setup shell script for setting up environment
* Fix arm/disarm commands to be required (bypassed by current takeoff and land commands)
* Mission commands

## Tested with
* DJI N3 flight controller