cmake_minimum_required(VERSION 3.0.2)
project(adjust_actuator_pose)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  std_msgs
  std_srvs
  behaviortree_cpp_v3
)

catkin_package(
  CATKIN_DEPENDS roscpp sensor_msgs std_msgs std_srvs behaviortree_cpp_v3
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)


add_executable(anticlockwise_node src/main2.cpp)
target_link_libraries(anticlockwise_node ${catkin_LIBRARIES})
add_executable(adjust_and_control_node src/main.cpp)
target_link_libraries(adjust_and_control_node ${catkin_LIBRARIES})

