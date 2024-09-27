cmake_minimum_required(VERSION 3.0.2)
project(needle_navigation)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf
  moveit_core
  moveit_ros_planning_interface
  moveit_visual_tools
)

# find_package(Boost REQUIRED COMPONENTS system)

catkin_package(
  CATKIN_DEPENDS roscpp rospy sensor_msgs std_msgs tf moveit_core moveit_ros_planning_interface moveit_visual_tools
  # DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

link_directories(/opt/ros/noetic/lib)

catkin_install_python(PROGRAMS
  src/move_pose.py  
  src/pose.py       
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


