cmake_minimum_required(VERSION 3.5)
project(jvl_motor)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# Install Python package
ament_python_install_package(${PROJECT_NAME})

# Install Python executables
install(PROGRAMS
  jvl_motor/jvl_motor_node.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
