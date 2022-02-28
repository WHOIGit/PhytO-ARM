cmake_minimum_required(VERSION 2.8.3)
project(ifcb)

find_package(
  catkin REQUIRED COMPONENTS
    message_generation
    std_msgs
    ds_core_msgs
)

add_service_files(
  FILES
  Command.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  ds_core_msgs
)

catkin_package()
