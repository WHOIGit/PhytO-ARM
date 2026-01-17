cmake_minimum_required(VERSION 3.5)
project(dli_power_switch)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# Install Python package
ament_python_install_package(${PROJECT_NAME})

# Install Python executables
install(PROGRAMS
  dli_power_switch/dli_power_switch_node.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
