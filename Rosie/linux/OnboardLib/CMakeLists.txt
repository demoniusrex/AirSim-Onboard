cmake_minimum_required(VERSION 3.5.0)
project(OnboardLib)

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


file(GLOB_RECURSE ${PROJECT_NAME}_sources 
  ${AIRSIM_ROOT}/${PROJECT_NAME}/src/api/*.cpp
  ${AIRSIM_ROOT}/${PROJECT_NAME}/src/common/common_utils/*.cpp
  ${AIRSIM_ROOT}/${PROJECT_NAME}/src/controllers/*.cpp
  ${AIRSIM_ROOT}/${PROJECT_NAME}/src/safety/*.cpp
  ${AIRSIM_ROOT}/${PROJECT_NAME}/src/vehicles/car/api/*.cpp
  ${AIRSIM_ROOT}/${PROJECT_NAME}/src/vehicles/multirotor/api/*.cpp
  ${AIRSIM_ROOT}/${PROJECT_NAME}/src/vehicles/multirotor/controllers/*.cpp
)

add_library(${PROJECT_NAME} STATIC ${${PROJECT_NAME}_sources})

target_link_libraries(${PROJECT_NAME} LINK_PUBLIC AirLib)
target_link_libraries(${PROJECT_NAME} LINK_PUBLIC MavLinkCom)
target_link_libraries(${PROJECT_NAME} LINK_PUBLIC ${RPC_LIB})
target_link_libraries(${PROJECT_NAME} LINK_PUBLIC djiosdk-core)
