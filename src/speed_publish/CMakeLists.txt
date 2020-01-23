cmake_minimum_required(VERSION 2.8.3)
project(speed_publish)


include_directories(${CMAKE_CURRENT_SOURCE_DIR}/lib)

add_executable(speed_publish src/receivetest.c src/common.c lib/src/libpcan.c)
target_link_libraries(speed_publish ${catkin_LIBRARIES})
target_compile_definitions(speed_publish PRIVATE "NO_RT")

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
