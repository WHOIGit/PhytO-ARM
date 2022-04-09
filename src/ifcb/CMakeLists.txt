cmake_minimum_required(VERSION 2.8.3)
project(ifcb)

# Mind the ordering!
# https://wiki.ros.org/catkin/CMakeLists.txt#Overall_Structure_and_Ordering

find_package(
  catkin REQUIRED COMPONENTS
    message_generation
    std_msgs
    ds_core_msgs
)

catkin_python_setup()

add_service_files(
  FILES
  Command.srv
  RunRoutine.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  ds_core_msgs
)

catkin_package()

catkin_install_python(
  PROGRAMS nodes/ifcb
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
