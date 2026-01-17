cmake_minimum_required(VERSION 3.5)
project(jvl_motor)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# Install Python package
ament_python_install_package(${PROJECT_NAME})

# Install Python executables
install(PROGRAMS
  jvl_motor/jvl_motor_node.py
  jvl_motor/mac400_sim_node.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(${PROJECT_NAME}_tests tests)
endif()

ament_package()
