cmake_minimum_required(VERSION 3.5.0)
project(OnboardServer)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -pthread -g -O0")

include_directories(${ROSIE_AIRSIM_ROOT}/DroneServer)
include_directories(${ROSIE_AIRSIM_ROOT}/MavLinkCom/include)
include_directories(${ROSIE_AIRSIM_ROOT}/${RPC_LIB_INCLUDES})
include_directories(${ROSIE_AIRSIM_ROOT}/AirLib/include)

include_directories(${ROSIE_ONBOARDSDK_SOURCE}/api/inc)
include_directories(${ROSIE_ONBOARDSDK_SOURCE}/utility/inc)
include_directories(${ROSIE_ONBOARDSDK_SOURCE}/hal/inc)
include_directories(${ROSIE_ONBOARDSDK_SOURCE}/protocol/inc)
include_directories(${ROSIE_ONBOARDSDK_SOURCE}/platform/linux/inc)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../common)

FILE(GLOB SOURCE_FILES *.hpp *.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/../common/dji_linux_environment.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/../common/dji_linux_helpers.cpp
        )

#add_executable(${PROJECT_NAME} ${SOURCE_FILES})

target_link_libraries(${PROJECT_NAME} LINK_PUBLIC AirLib)
target_link_libraries(${PROJECT_NAME} LINK_PUBLIC MavLinkCom)
target_link_libraries(${PROJECT_NAME} LINK_PUBLIC ${RPC_LIB})
target_link_libraries(${PROJECT_NAME} LINK_PUBLIC djiosdk-core)




