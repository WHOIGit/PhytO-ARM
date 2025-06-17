cmake_minimum_required(VERSION 2.8.3)
project(dli_power_switch)

find_package(
  catkin REQUIRED COMPONENTS
    message_generation
    std_msgs
    ds_core_msgs
)

add_message_files(
  FILES
  OutletStatus.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  ds_core_msgs
)

catkin_package()
