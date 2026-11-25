cmake_minimum_required(VERSION 3.10)
project(phyto_gazebo)

find_package(catkin REQUIRED COMPONENTS geometry_msgs roscpp std_msgs)
find_package(gazebo REQUIRED)

catkin_package()
link_directories(${GAZEBO_LIBRARY_DIRS})
add_library(phyto_tether SHARED plugins/tether.cpp)
target_compile_features(phyto_tether PRIVATE cxx_std_17)
target_include_directories(phyto_tether SYSTEM PRIVATE
  ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS})
target_link_libraries(phyto_tether ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

catkin_install_python(PROGRAMS scripts/ctd_bridge.py scripts/winch_drum.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(TARGETS phyto_tether
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
install(DIRECTORY worlds DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

if(CATKIN_ENABLE_TESTING)
  find_package(ds_sensor_msgs REQUIRED)
  catkin_add_nosetests(test/ctd_bridge_tests.py DEPENDENCIES
    ${ds_sensor_msgs_EXPORTED_TARGETS})
  catkin_add_nosetests(test/winch_drum_tests.py)
endif()
