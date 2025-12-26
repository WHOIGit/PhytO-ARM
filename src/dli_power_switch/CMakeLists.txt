cmake_minimum_required(VERSION 2.8.3)
project(dli_power_switch)

find_package(
  catkin REQUIRED COMPONENTS
    message_generation
    std_msgs
    ds_core_msgs
)

catkin_python_setup()

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

catkin_install_python(
  PROGRAMS src/dli_power_switch_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

if (CATKIN_ENABLE_TESTING)
  catkin_add_nosetests(tests)
endif()
