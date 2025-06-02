cmake_minimum_required(VERSION 2.8.3)
project(phyto_arm)

find_package(
  catkin REQUIRED COMPONENTS
    message_generation
    actionlib_msgs
    sensor_msgs
    std_msgs
    ds_core_msgs
)

add_action_files(
  DIRECTORY action
  FILES
  MoveToDepth.action
  RunIFCB.action
)

add_message_files(
  FILES
  ConductorState.msg
  ConductorStates.msg
  DepthProfile.msg
)

add_service_files(
  FILES
  LockOperation.srv
  LockCheck.srv
)

generate_messages(
  DEPENDENCIES
  actionlib_msgs
  std_msgs
  ds_core_msgs
)

catkin_package()

# Install all node executables
catkin_install_python(
  PROGRAMS
    src/arm_base.py
    src/arm_chanos.py
    src/arm_ifcb.py
    src/ifcb_runner.py
    src/lock_manager.py
    src/network_data_capture.py
    src/profiler_node.py
    src/web_node.py
    src/winch_node.py
    src/mocks/mock_ctd.py
    src/mocks/mock_ifcb_runner.py
    src/mocks/mock_profiler_node.py
    src/mocks/mock_winch_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Install launch files
install(
  DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  FILES_MATCHING PATTERN "*.launch"
)

if (CATKIN_ENABLE_TESTING)
  catkin_add_nosetests(src/tests.py)
endif()
