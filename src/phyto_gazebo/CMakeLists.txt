cmake_minimum_required(VERSION 3.10)
project(phyto_gazebo)

find_package(ament_cmake REQUIRED)
find_package(gazebo_dev REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

link_directories(${gazebo_dev_LIBRARY_DIRS})
add_library(phyto_tether SHARED plugins/tether.cpp)
target_compile_features(phyto_tether PRIVATE cxx_std_17)
ament_target_dependencies(phyto_tether gazebo_dev geometry_msgs rclcpp std_msgs)

install(PROGRAMS scripts/view_winch DESTINATION lib/${PROJECT_NAME})
install(TARGETS phyto_tether LIBRARY DESTINATION lib)
install(DIRECTORY launch worlds DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(launch_testing_ament_cmake REQUIRED)
  add_launch_test(test/ifcb_mock_test.py TIMEOUT 45
    APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}/test)
  add_launch_test(test/gazebo_winch_test.py TIMEOUT 240
    APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}/test)
endif()

ament_package()
