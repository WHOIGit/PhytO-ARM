cmake_minimum_required(VERSION 3.10)
project(phyto_sim_utils)

find_package(catkin REQUIRED)
catkin_package()

install(PROGRAMS scripts/wait_for_socket
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/scripts)
