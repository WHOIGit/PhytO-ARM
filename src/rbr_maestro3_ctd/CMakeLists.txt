cmake_minimum_required(VERSION 2.8.3)
project(rbr_maestro3_ctd)

find_package(
  catkin REQUIRED COMPONENTS
    message_generation
    std_msgs
    ds_core_msgs
    ds_sensor_msgs
)

add_message_files(
  FILES
  RbrMeasurement.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  ds_core_msgs
)

catkin_package()
