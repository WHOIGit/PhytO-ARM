cmake_minimum_required(VERSION 3.5)
project(dli_power_switch)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(ds_core_msgs REQUIRED)

# Generate messages
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/OutletStatus.msg"
  DEPENDENCIES std_msgs ds_core_msgs
)

# Install Python package
ament_python_install_package(${PROJECT_NAME})

# Install Python executables
install(PROGRAMS
  dli_power_switch/dli_power_switch_node.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
