cmake_minimum_required(VERSION 3.5)
project(ifcb_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Generate services
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/Command.srv"
  "srv/RunRoutine.srv"
)

ament_package()
