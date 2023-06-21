cmake_minimum_required(VERSION 2.8.3)
project(triton_classifier)

find_package(
  catkin REQUIRED COMPONENTS
    message_generation
    std_msgs
)

add_message_files(
  FILES
  Classification.msg
  ObjectHypothesisWithClassName.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package()
