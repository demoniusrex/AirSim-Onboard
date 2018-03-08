cmake_minimum_required(VERSION 3.5.0)
project(OnboardServer)
cmake_policy(SET CMP0022 NEW)

LIST(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/../../../AirSim/cmake/cmake-modules") 
INCLUDE("${CMAKE_CURRENT_LIST_DIR}/../../../AirSim/cmake/cmake-modules/CommonSetup.cmake")

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -pthread -g -O0")

IncludeEigen()

include_directories(${ROSIE_AIRSIM_SOURCE}/DroneServer)
include_directories(${ROSIE_AIRSIM_SOURCE}/MavLinkCom/include)
include_directories(${ROSIE_AIRSIM_SOURCE}/${RPC_LIB_INCLUDES})
include_directories(${ROSIE_AIRSIM_SOURCE}/AirLib/include)

include_directories(${ROSIE_ONBOARDSDK_SOURCE}/api/inc)
include_directories(${ROSIE_ONBOARDSDK_SOURCE}/utility/inc)
include_directories(${ROSIE_ONBOARDSDK_SOURCE}/hal/inc)
include_directories(${ROSIE_ONBOARDSDK_SOURCE}/protocol/inc)
include_directories(${ROSIE_ONBOARDSDK_SOURCE}/platform/linux/inc)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../OnboardLib/include)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../common)

FILE(GLOB SOURCE_FILES *.hpp *.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/../common/dji_linux_environment.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/../common/dji_linux_helpers.cpp
        )

add_executable(${PROJECT_NAME} ${SOURCE_FILES})

CommonTargetLink()
target_link_libraries(${PROJECT_NAME} djiosdk-core)
target_link_libraries(${PROJECT_NAME} ${RPC_LIB})
target_link_libraries(${PROJECT_NAME} AirLib)
target_link_libraries(${PROJECT_NAME} MavLinkCom)




