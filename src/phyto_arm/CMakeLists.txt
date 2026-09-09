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

if (CATKIN_ENABLE_TESTING)
  catkin_add_nosetests(tests)
endif()
