cmake_minimum_required(VERSION 2.8.3)
project(joint_controller)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  rospy
  rosconsole
  dynamixel_workbench_msgs
)

catkin_package(
  CATKIN_DEPENDS dynamixel_workbench_msgs
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(joint_controller src/joint_controller.cpp)
target_link_libraries(joint_controller ${catkin_LIBRARIES})
