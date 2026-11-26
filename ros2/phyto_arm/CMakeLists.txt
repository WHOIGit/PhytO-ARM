cmake_minimum_required(VERSION 3.5)
project(phyto_arm)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})
install(PROGRAMS ../../phyto-arm DESTINATION bin)
install(FILES ../../phyto-arm DESTINATION ${PYTHON_INSTALL_DIR}/${PROJECT_NAME} RENAME deployment.py)
install(FILES ../../configs/example.yaml DESTINATION share/${PROJECT_NAME}/configs)

# Install Python executables
install(PROGRAMS
  phyto_arm/arm_base.py
  phyto_arm/arm_chanos.py
  phyto_arm/arm_ifcb.py
  phyto_arm/ifcb_runner.py
  phyto_arm/lock_manager.py
  phyto_arm/network_data_capture.py
  phyto_arm/profiler_node.py
  phyto_arm/seatrac_starlink_manager.py
  phyto_arm/web_node.py
  phyto_arm/winch_node.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
  FILES_MATCHING PATTERN "*.launch.xml" PATTERN "*.launch.py" PATTERN "*.params.yaml"
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  foreach(module arm_chanos arm_ifcb lock_manager network_data_capture
                 seatrac_starlink_manager)
    ament_add_pytest_test(${module}_tests tests/${module}_tests.py)
  endforeach()
endif()

ament_package()
