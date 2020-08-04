cmake_minimum_required(VERSION 3.0.2)
project(py_image_saver)

find_package(catkin REQUIRED COMPONENTS
  rospy
  sensor_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
