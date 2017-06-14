cmake_minimum_required(VERSION 2.8.3)
project(ed_recognition_msgs)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  sensor_msgs
)

# Generate messages in the 'msg' folder
add_message_files(
  FILES
  EntityInfo.msg
)

# Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS message_generation geometry_msgs
)
