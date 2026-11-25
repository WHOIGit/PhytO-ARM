cmake_minimum_required(VERSION 3.10)
project(phyto_sim_utils)

find_package(catkin REQUIRED)
catkin_package()

install(PROGRAMS scripts/wait_for_socket
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/scripts)

catkin_install_python(PROGRAMS
  scripts/ctd_sim.py
  scripts/winch_drum.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

if(CATKIN_ENABLE_TESTING)
  find_package(ds_sensor_msgs REQUIRED)
  catkin_add_nosetests(test/ctd_sim_tests.py DEPENDENCIES
    ${ds_sensor_msgs_EXPORTED_TARGETS})
  catkin_add_nosetests(test/winch_drum_tests.py)
endif()
