cmake_minimum_required(VERSION 3.10)
project(phyto_sim_utils)

find_package(catkin REQUIRED)
catkin_python_setup()
catkin_package()

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(PROGRAMS scripts/arm_config scripts/wait_for_socket
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/scripts)

catkin_install_python(PROGRAMS
  scripts/ctd_sim.py
  scripts/gps_sim.py
  scripts/gpsd_client_mock
  scripts/vertical_line.py
  scripts/winch_drum.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

if(CATKIN_ENABLE_TESTING)
  find_package(ds_sensor_msgs REQUIRED)
  find_package(rostest REQUIRED)
  add_rostest(test/gpsd_client_mock.test)
  catkin_add_nosetests(test/ctd_sim_tests.py DEPENDENCIES
    ${ds_sensor_msgs_EXPORTED_TARGETS})
  catkin_add_nosetests(test/winch_drum_tests.py)
endif()
