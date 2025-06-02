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

catkin_install_python(PROGRAMS scripts/rbr_sim.py src/rbr_maestro3_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  add_rostest(test/rbr_sim.test DEPENDENCIES ${${PROJECT_NAME}_EXPORTED_TARGETS})
endif()
