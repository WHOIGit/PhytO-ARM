cmake_minimum_required(VERSION 3.5)
project(rbr_maestro3_ctd)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# Install Python package
ament_python_install_package(${PROJECT_NAME})

# Install Python executables
install(PROGRAMS
  rbr_maestro3_ctd/rbr_maestro3_node.py
  scripts/rbr_sim.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch)

if(BUILD_TESTING)
  find_package(launch_testing_ament_cmake REQUIRED)
  add_launch_test(test/rbr_sim_test.py TIMEOUT 25)
endif()

ament_package()
