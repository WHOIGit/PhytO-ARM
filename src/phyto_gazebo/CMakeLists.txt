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

install(TARGETS phyto_tether
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
install(DIRECTORY worlds DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
