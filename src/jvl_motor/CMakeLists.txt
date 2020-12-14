cmake_minimum_required(VERSION 2.8.3)
project(jvl_motor)

find_package(
  catkin REQUIRED COMPONENTS
    message_generation
    std_msgs
    ds_core_msgs
)

add_message_files(
  FILES
  Electrical.msg
  Error.msg
  Motion.msg
)

add_service_files(
  FILES
  MoveCmd.srv
  StopCmd.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  ds_core_msgs
)

catkin_package()
