cmake_minimum_required(VERSION 3.5)
project(jvl_motor_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(ds_core_msgs REQUIRED)

# Generate messages and services
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Electrical.msg"
  "msg/Error.msg"
  "msg/Motion.msg"
  "srv/SetPositionCmd.srv"
  "srv/SetPositionEnvelopeCmd.srv"
  "srv/SetVelocityCmd.srv"
  "srv/StopCmd.srv"
  "srv/ZeroCmd.srv"
  DEPENDENCIES std_msgs ds_core_msgs
)

ament_package()
