// A massless winch line pulls at the lifting eye when it is stretched
#include <algorithm>
#include <cmath>
#include <chrono>
#include <functional>
#include <memory>
#include <vector>

#include <geometry_msgs/msg/point_stamped.hpp>
#include <ignition/gazebo/Link.hh>
#include <ignition/gazebo/Model.hh>
#include <ignition/gazebo/System.hh>
#include <ignition/gazebo/Util.hh>
#include <ignition/gazebo/World.hh>
#include <ignition/gazebo/components/Pose.hh>
#include <ignition/msgs/Utility.hh>
#include <ignition/msgs/marker.pb.h>
#include <ignition/plugin/Register.hh>
#include <ignition/transport/Node.hh>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>

namespace phyto_gazebo
{
namespace sim = ignition::gazebo;
using ignition::math::Pose3d;
using ignition::math::Vector3d;
using SimTime = std::chrono::steady_clock::duration;

class Tether : public sim::System, public sim::ISystemConfigure, public sim::ISystemPreUpdate
{
  public: void Configure(const sim::Entity &entity, const std::shared_ptr<const sdf::Element> &sdf,
                         sim::EntityComponentManager &ecm, sim::EventManager &) override
  {
    // The probe collision locates the CTD within the moving payload
    body = sim::Link(sim::Model(entity).LinkByName(ecm, "body"));
    const auto probe = body.CollisionByName(ecm, "probe");
    if (probe == sim::kNullEntity)
    {
      ignerr << "The tether needs the payload and CTD probe\n";
      return;
    }
    probeOffset = ecm.Component<sim::components::Pose>(probe)->Data().Pos();
    body.EnableVelocityChecks(ecm);
    gravity = sim::World(sim::worldEntity(ecm)).Gravity(ecm).value();
    drumPose = sdf->Get<Pose3d>("drum_pose");

    // The anchor is fixed in the world and the lifting eye moves with the body
    anchor = sdf->Get<Vector3d>("anchor");
    eye = sdf->Get<Vector3d>("lifting_eye");
    initialLength = sdf->Get<double>("initial_length");
    length = initialLength;
    circumference = sdf->Get<double>("spool_circumference");
    damping = sdf->Get<double>("damping");
    stiffness = sdf->Get<double>("stiffness");

    // Apply buoyancy and drag relative to a steady current
    current = sdf->Get<Vector3d>("current");
    displacedMass = sdf->Get<double>("displaced_mass");

    // Consume length commands on the physics thread at the start of each step
    context = std::make_shared<rclcpp::Context>();
    context->init(0, nullptr);
    node = std::make_shared<rclcpp::Node>("winch_tether", rclcpp::NodeOptions().context(context));
    rclcpp::ExecutorOptions options;
    options.context = context;
    executor = std::make_unique<rclcpp::executors::SingleThreadedExecutor>(options);
    executor->add_node(node);
    lineSub = node->create_subscription<std_msgs::msg::Float64>(
        "sim/line_length", 1, std::bind(&Tether::SetLength, this, std::placeholders::_1));

    // Recover the last drum position if it was published before the world loaded.
    initialLineSub = node->create_subscription<std_msgs::msg::Float64>(
        "sim/line_length", rclcpp::QoS(1).transient_local(),
        [this](std_msgs::msg::Float64::ConstSharedPtr msg) {
          if (initialLineSub)
            SetLength(msg);
        });
    ctdPub = node->create_publisher<geometry_msgs::msg::PointStamped>(
        "sim/ctd_position", 1);
  }

  private: void SetLength(std_msgs::msg::Float64::ConstSharedPtr msg)
  {
    if (std::isfinite(msg->data) && msg->data >= 0)
    {
      initialLineSub.reset();
      length = msg->data;
    }
  }

  public: void PreUpdate(const sim::UpdateInfo &info, sim::EntityComponentManager &ecm) override
  {
    if (!executor || info.paused)
      return;
    executor->spin_some();
    const auto pose = sim::worldPose(body.Entity(), ecm);
    const auto liftingEye = pose.Pos() + pose.Rot().RotateVector(eye);
    const auto span = liftingEye - anchor;
    const double distance = span.Length();

    // Slack line exerts no force. Once taut, the line pulls along the span
    // and the damper opposes the lifting eye's speed along that direction.
    if (distance > length)
    {
      const auto direction = span / distance;
      const double speed = direction.Dot(body.WorldLinearVelocity(ecm, eye).value());
      const double tension = std::max(0.0, stiffness * (distance - length)
                                            + damping * speed);
      body.AddWorldForce(ecm, -tension * direction, eye);
    }

    // Approximate the submerged part of the upright 1.02 m housing
    const double submerged = std::clamp((0.51 - pose.Pos().Z()) / 1.02,
                                        0.0, 1.0);
    const auto velocity = body.WorldLinearVelocity(ecm).value() - current;
    const auto drag = -(18.0 + 32.0 * velocity.Length()) * velocity;
    const auto buoyancy = -displacedMass * gravity;
    body.AddWorldWrench(ecm, submerged * (buoyancy + drag),
                        -5.0 * submerged * body.WorldAngularVelocity(ecm).value());

    // Sample the probe at 10 Hz and draw the line at 25 Hz in simulated time
    if (Due(info.simTime, lastSample, 0.1))
    {
      geometry_msgs::msg::PointStamped sample;
      sample.header.stamp = rclcpp::Time(std::chrono::duration_cast<std::chrono::nanoseconds>(info.simTime).count());
      sample.header.frame_id = "world";
      const auto probe = pose.Pos() + pose.Rot().RotateVector(probeOffset);
      sample.point.x = probe.X();
      sample.point.y = probe.Y();
      sample.point.z = probe.Z();
      ctdPub->publish(sample);
      lastSample = info.simTime;
    }

    // The physics runs at full rate even when no viewer is connected
    if (!Due(info.simTime, lastVisual, 0.04))
      return;
    lastVisual = info.simTime;
    std::vector<ignition::transport::ServicePublisher> viewers;
    if (!transport.ServiceInfo("/marker", viewers))
      return;

    DrawLine(liftingEye, span, distance);
    const double angle = 2.0 * M_PI * (length - initialLength) / circumference;
    auto stripe = drumPose;
    stripe.Rot() = ignition::math::Quaterniond(0, angle, 0) * stripe.Rot();
    DrawMarker(3, ignition::msgs::Marker::BOX, stripe, {0.035, 0.004, 0.33},
               {0.89f, 0.91f, 0.86f});
  }

  // Publish immediately when Gazebo rewinds the clock
  private: static bool Due(SimTime now, SimTime last, double period)
  {
    return now < last || now - last >= std::chrono::duration<double>(period);
  }

  // The two visible halves bow out when the paid-out line is slack
  private: void DrawLine(const Vector3d &end, const Vector3d &span,
                         double distance)
  {
    Vector3d elbow = (anchor + end) * 0.5;
    if (length > distance)
    {
      const auto direction = span.Normalized();
      Vector3d bend(0.4, 0, -1);
      bend -= direction * bend.Dot(direction);
      if (bend.Length() < 1e-6)
        bend = Vector3d::UnitY;
      const double bow = 0.5 * std::sqrt((length - distance)
                                         * (length + distance));
      elbow += bend.Normalized() * bow;
    }
    DrawSegment(1, anchor, elbow);
    DrawSegment(2, elbow, end);
  }

  private: void DrawSegment(unsigned int id,
                            const Vector3d &a, const Vector3d &b)
  {
    // A Gazebo cylinder is centered on its local Z axis
    const auto delta = b - a;
    const auto center = (a + b) * 0.5;
    const double pitch = std::atan2(std::hypot(delta.X(), delta.Y()),
                                    delta.Z());
    const double yaw = std::atan2(delta.Y(), delta.X());

    DrawMarker(id, ignition::msgs::Marker::CYLINDER,
               Pose3d(center, ignition::math::Quaterniond(0, pitch, yaw)),
               {0.018, 0.018, std::max(delta.Length(), 1e-6)}, {0.99f, 0.79f, 0.3f});
  }

  private: void DrawMarker(unsigned int id, ignition::msgs::Marker::Type type,
                           const Pose3d &pose, const Vector3d &scale,
                           const ignition::math::Color &color)
  {
    ignition::msgs::Marker marker;
    marker.set_ns("phyto_winch");
    marker.set_id(id);
    marker.set_type(type);
    marker.set_action(ignition::msgs::Marker::ADD_MODIFY);
    ignition::msgs::Set(marker.mutable_pose(), pose);
    ignition::msgs::Set(marker.mutable_scale(), scale);
    ignition::msgs::Set(marker.mutable_material()->mutable_ambient(), color);
    ignition::msgs::Set(marker.mutable_material()->mutable_diffuse(), color);
    transport.Request("/marker", marker);
  }

  private: sim::Link body;
  private: Pose3d drumPose;
  private: Vector3d anchor, current, eye, gravity, probeOffset;
  private: double circumference, damping, displacedMass, initialLength;
  private: double length, stiffness;
  private: SimTime lastSample{}, lastVisual{};
  private: rclcpp::Context::SharedPtr context;
  private: rclcpp::Node::SharedPtr node;
  private: std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> executor;
  private: rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr lineSub, initialLineSub;
  private: rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr ctdPub;
  private: ignition::transport::Node transport;
};
}

IGNITION_ADD_PLUGIN(phyto_gazebo::Tether, ignition::gazebo::System,
                    phyto_gazebo::Tether::ISystemConfigure, phyto_gazebo::Tether::ISystemPreUpdate)
IGNITION_ADD_PLUGIN_ALIAS(phyto_gazebo::Tether, "winch_tether")
