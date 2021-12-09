cmake_minimum_required(VERSION 2.8.3)
project(aml_ctd)

find_package(
  catkin REQUIRED COMPONENTS
    std_msgs
    ds_core_msgs
    ds_sensor_msgs
)

catkin_package()
