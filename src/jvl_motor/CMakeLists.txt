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

install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  find_package(launch_testing_ament_cmake REQUIRED)
  ament_add_pytest_test(${PROJECT_NAME}_tests tests)
  add_launch_test(test/mac400_driver_test.py TIMEOUT 60)
endif()

ament_package()
