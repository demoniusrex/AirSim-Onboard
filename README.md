# AirSim-Onboard
Microsoft AirSim multi-rotor implementation for DJI Onboard-SDK compatible flight controllers

_Work in progress_

Milestone: v0.3

## Details 
* Linux Only. DJI Onboard SDK not available for Windows
* Requires GNU C++ Compiler (Tested with G++-6 and G++-7)
* Provides new Onboard-SDK compatible multi-rotor API implementation
* Works with DroneShell RPC client. Should work with other clients.
* Forked DroneServer to work with new multirotor API
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

## Tested with
* DJI N3 flight controller