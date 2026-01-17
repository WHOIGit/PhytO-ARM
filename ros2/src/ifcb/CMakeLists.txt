cmake_minimum_required(VERSION 3.5)
project(ifcb)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

# Generate services
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/Command.srv"
  "srv/RunRoutine.srv"
)

# Install Python package
ament_python_install_package(${PROJECT_NAME})

# Install Python executables
install(PROGRAMS
  scripts/ifcb
  scripts/ifcb_logfilter
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
