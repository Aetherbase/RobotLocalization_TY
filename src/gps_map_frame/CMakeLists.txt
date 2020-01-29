cmake_minimum_required(VERSION 2.8.3)
project(gps_map_frame)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  serial
  sensor_msgs
  std_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES gps_map_frame
#  CATKIN_DEPENDS sensor_msgs std_msgs
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(speedWrite src/send_speed.cpp)
target_link_libraries(speedWrite ${catkin_LIBRARIES})
#add_dependencies(talker beginner_tutorials_generate_messages_cpp)
