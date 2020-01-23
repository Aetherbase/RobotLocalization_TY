cmake_minimum_required(VERSION 2.8.3)
project(speed_publish)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/lib)

add_executable(speed_publish src/speed_publish.cpp lib/src/common.c lib/src/libpcan.c)
target_link_libraries(speed_publish ${catkin_LIBRARIES})


catkin_package(
  INCLUDE_DIRS lib include
# LIBRARIES faro_can_sdk
#  CATKIN_DEPENDS roscpp
#  DEPENDS system_lib
)

include_directories(
 lib include 
  ${catkin_INCLUDE_DIRS}
)
