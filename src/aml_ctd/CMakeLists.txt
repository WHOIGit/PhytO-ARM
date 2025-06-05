cmake_minimum_required(VERSION 2.8.3)
project(aml_ctd)

find_package(
  catkin REQUIRED COMPONENTS
    message_generation
    std_msgs
    ds_core_msgs
    ds_sensor_msgs
)

add_message_files(
  FILES
  AmlMeasurement.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  ds_core_msgs
)

catkin_package()

if (CATKIN_ENABLE_TESTING)
  catkin_add_nosetests(tests)
endif()
