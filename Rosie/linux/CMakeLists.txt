cmake_minimum_required(VERSION 3.5.0)
project(rosie-linux)

if(NOT ROSIE_ONBOARDSDK_SOURCE)
    set(ROSIE_ONBOARDSDK_SOURCE "${CMAKE_CURRENT_SOURCE_DIR}/../../Onboard-SDK/osdk-core")
endif()

if(NOT ROSIE_AIRSIM_SOURCE)
    set(ROSIE_AIRSIM_SOURCE "${CMAKE_CURRENT_SOURCE_DIR}/../../AirSim")
endif()

add_subdirectory(OnboardServer)
