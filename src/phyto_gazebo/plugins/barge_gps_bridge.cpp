// Publish both the fix and the ENU velocity from the barge's NavSat sensor.
#include <memory>

#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <ignition/msgs/navsat.pb.h>
#include <ignition/transport/Node.hh>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("barge_gps_bridge");
  auto qos = rclcpp::SensorDataQoS().reliable();
  auto fixes = node->create_publisher<sensor_msgs::msg::NavSatFix>("sim/gps", qos);
  auto velocities = node->create_publisher<geometry_msgs::msg::Vector3Stamped>("sim/gps_velocity", qos);

  ignition::transport::Node transport;
  transport.Subscribe<ignition::msgs::NavSat>("/sim/gps", [&](const ignition::msgs::NavSat &msg) {
    sensor_msgs::msg::NavSatFix fix;
    fix.header.stamp = rclcpp::Time(msg.header().stamp().sec(), msg.header().stamp().nsec());
    fix.header.frame_id = "barge";
    fix.status.status = sensor_msgs::msg::NavSatStatus::STATUS_FIX;
    fix.status.service = sensor_msgs::msg::NavSatStatus::SERVICE_GPS;
    fix.latitude = msg.latitude_deg();
    fix.longitude = msg.longitude_deg();
    fix.altitude = msg.altitude();
    fix.position_covariance_type = sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;
    fixes->publish(fix);

    geometry_msgs::msg::Vector3Stamped velocity;
    velocity.header = fix.header;
    velocity.vector.x = msg.velocity_east();
    velocity.vector.y = msg.velocity_north();
    velocity.vector.z = msg.velocity_up();
    velocities->publish(velocity);
  });
  rclcpp::spin(node);
  rclcpp::shutdown();
}
