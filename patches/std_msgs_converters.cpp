// Custom converters for std_msgs/Duration and std_msgs/Time
// These provide template function specializations needed by auto-generated bridge code
// See https://github.com/ros2/ros1_bridge/issues/464

#include "ros1_bridge/convert_builtin_interfaces.hpp"
#include <std_msgs/Duration.h>
#include <std_msgs/Time.h>
#include <builtin_interfaces/msg/duration.hpp>
#include <builtin_interfaces/msg/time.hpp>

namespace ros1_bridge {

// std_msgs/Duration converters
template<>
void convert_1_to_2(
  const std_msgs::Duration & ros1_msg,
  builtin_interfaces::msg::Duration & ros2_msg)
{
  ros2_msg.sec = ros1_msg.data.sec;
  ros2_msg.nanosec = ros1_msg.data.nsec;
}

template<>
void convert_2_to_1(
  const builtin_interfaces::msg::Duration & ros2_msg,
  std_msgs::Duration & ros1_msg)
{
  ros1_msg.data.sec = ros2_msg.sec;
  ros1_msg.data.nsec = ros2_msg.nanosec;
}

// std_msgs/Time converters
template<>
void convert_1_to_2(
  const std_msgs::Time & ros1_msg,
  builtin_interfaces::msg::Time & ros2_msg)
{
  ros2_msg.sec = ros1_msg.data.sec;
  ros2_msg.nanosec = ros1_msg.data.nsec;
}

template<>
void convert_2_to_1(
  const builtin_interfaces::msg::Time & ros2_msg,
  std_msgs::Time & ros1_msg)
{
  ros1_msg.data.sec = ros2_msg.sec;
  ros1_msg.data.nsec = ros2_msg.nanosec;
}

}  // namespace ros1_bridge
