cmake_minimum_required(VERSION 2.8.3)
project(imuread)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  std_msgs
)

add_compile_options(-std=c++11)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)

link_directories(${PROJECT_BINARY_DIR}/lib)

set (FARO_CAN_SDK_DEBUG_DEF "FARO_CAN_SDK_DEBUG=1")

# exectuabe version
set (FARO_CAN_SDK_VERSION_MAJOR 3)
set (FARO_CAN_SDK_VERSION_MINOR 1)
set (FARO_CAN_SDK_VERSION_PATCH 16)

configure_file (
	"${CMAKE_CURRENT_SOURCE_DIR}/template/faro_can_sdk_demo_sensor_config.h.in"
	"${CMAKE_CURRENT_SOURCE_DIR}/include/faro_can_sdk_demo_sensor_config.h"
)

add_executable(imu-publish src/publish_imu.cpp)
target_link_libraries(imu-publish ${PROJECT_SOURCE_DIR}/lib/libfaro_can_sdk.so.3 ${catkin_LIBRARIES})

### comment below to enable release mode and disable debug mode 
target_compile_definitions(imu-publish PRIVATE ${FARO_CAN_SDK_DEBUG_DEF})

add_executable(demosensor src/faro_can_sdk_demo_sensor.c)
target_link_libraries(demosensor rt pthread ${PROJECT_SOURCE_DIR}/lib/libfaro_can_sdk.so.3 ${catkin_LIBRARIES})
target_compile_definitions(demosensor PRIVATE ${FARO_CAN_SDK_DEBUG_DEF})

catkin_package(
#  INCLUDE_DIRS include
 LIBRARIES faro_can_sdk
#  CATKIN_DEPENDS roscpp
#  DEPENDS system_lib
)

include_directories(
 include
  ${catkin_INCLUDE_DIRS}
)
