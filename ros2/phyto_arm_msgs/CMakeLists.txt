cmake_minimum_required(VERSION 3.5)
project(phyto_arm_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(action_msgs REQUIRED)

# Generate messages, services, and actions
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ConductorState.msg"
  "msg/ConductorStates.msg"
  "msg/DepthProfile.msg"
  "srv/LockCheck.srv"
  "srv/LockOperation.srv"
  "action/MoveToDepth.action"
  "action/RunIFCB.action"
  DEPENDENCIES std_msgs builtin_interfaces action_msgs
)

ament_package()
