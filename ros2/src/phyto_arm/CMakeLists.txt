cmake_minimum_required(VERSION 3.5)
project(phyto_arm)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(action_msgs REQUIRED)

# Generate messages, services, and actions
# Note: Action files auto-generate Goal/Result/Feedback message types
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ConductorState.msg"
  "msg/ConductorStates.msg"
  "msg/DepthProfile.msg"
  "srv/LockCheck.srv"
  "srv/LockOperation.srv"
  "action/MoveToDepth.action"
  "action/RunIFCB.action"
  DEPENDENCIES std_msgs builtin_interfaces action_msgs
)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

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
  phyto_arm/mocks/mock_winch_node.py
  phyto_arm/mocks/mock_ifcb_runner.py
  phyto_arm/mocks/mock_ctd.py
  phyto_arm/mocks/mock_profiler_node.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_package()
