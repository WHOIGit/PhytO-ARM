cmake_minimum_required(VERSION 3.5)
project(phyto_sim_utils)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  scripts/arm_config
  scripts/gps_sim.py
  scripts/gpsd_client_mock
  scripts/run_ifcb_emulator
  scripts/wait_for_socket
  scripts/ctd_sim.py
  scripts/vertical_line.py
  scripts/winch_drum.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(ctd_sim_tests test/ctd_sim_tests.py)
  ament_add_pytest_test(parameters_tests test/parameters_tests.py)
  ament_add_pytest_test(winch_drum_tests test/winch_drum_tests.py)
endif()

ament_package()
