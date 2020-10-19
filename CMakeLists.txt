cmake_minimum_required(VERSION 3.0.2)
project(py_image_saver)

find_package(catkin REQUIRED COMPONENTS
  rospy
  message_generation
  sensor_msgs
)

add_service_files(
  FILES
  Int.srv
)

generate_messages(
  DEPENDENCIES
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
