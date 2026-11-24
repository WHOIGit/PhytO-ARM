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
  SetPositionCmd.srv
  SetPositionEnvelopeCmd.srv
  SetVelocityCmd.srv
  StopCmd.srv
  ZeroCmd.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  ds_core_msgs
)

catkin_package()

# The driver and simulator import the motor modules from their own directory
install(PROGRAMS src/jvl_motor_node.py src/mac400_sim_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(FILES src/mac400.py src/mac400_model.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(PROGRAMS scripts/wait_for_socket
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/scripts)

if (CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  catkin_add_nosetests(tests DEPENDENCIES ${PROJECT_NAME}_generate_messages_py)
  add_rostest(test/mac400_sim.test DEPENDENCIES
    ${PROJECT_NAME}_generate_messages_py
  )
endif()
