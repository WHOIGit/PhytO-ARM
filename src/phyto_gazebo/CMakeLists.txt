cmake_minimum_required(VERSION 3.10)
project(phyto_gazebo)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(ignition-gazebo6 REQUIRED)
find_package(ignition-transport11 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_library(phyto_tether SHARED plugins/tether.cpp)
target_compile_features(phyto_tether PRIVATE cxx_std_17)
target_link_libraries(phyto_tether ignition-gazebo6::ignition-gazebo6)
ament_target_dependencies(phyto_tether geometry_msgs rclcpp std_msgs)

add_executable(barge_gps_bridge plugins/barge_gps_bridge.cpp)
target_link_libraries(barge_gps_bridge ignition-transport11::core)
ament_target_dependencies(barge_gps_bridge geometry_msgs rclcpp sensor_msgs)

ament_environment_hooks(hooks/fortress.dsv)
install(TARGETS barge_gps_bridge DESTINATION lib/${PROJECT_NAME})
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
