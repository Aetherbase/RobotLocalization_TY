cmake_minimum_required(VERSION 2.8.3)
project (faro_can_sdk)

find_package(catkin REQUIRED COMPONENTS
  roscpp
)

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

add_executable(publish_imu src/publish_imu.cpp)
target_link_libraries(publish_imu rt pthread ${PROJECT_SOURCE_DIR}/lib/libfaro_can_sdk.so.3 ${catkin_LIBRARIES})

### comment below to enable release mode and disable debug mode 
target_compile_definitions(publish_imu PRIVATE ${FARO_CAN_SDK_DEBUG_DEF})

add_executable(demosensor_test src/faro_can_sdk_demo_sensor.c)
target_link_libraries(demosensor_test rt pthread ${PROJECT_SOURCE_DIR}/lib/libfaro_can_sdk.so.3 ${catkin_LIBRARIES})
target_compile_definitions(demosensor_test PRIVATE ${FARO_CAN_SDK_DEBUG_DEF})

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
