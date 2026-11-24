// A massless winch line pulls at the lifting eye when it is stretched
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>

#include <gazebo/common/Events.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>
#include <geometry_msgs/PointStamped.h>
#include <ros/callback_queue.h>
#include <ros/ros.h>
#include <std_msgs/Float64.h>

namespace gazebo
{
using ignition::math::Vector3d;

class Tether : public ModelPlugin
{
  public: void Load(physics::ModelPtr model, sdf::ElementPtr sdf) override
  {
    // The probe collision locates the CTD within the moving payload
    body = model->GetLink("body");
    const auto probe = body ? body->GetCollision("probe") : nullptr;
    if (!probe || !ros::isInitialized())
    {
      gzerr << "The tether needs the payload, CTD probe and gazebo_ros\n";
      return;
    }

    probeOffset = probe->RelativePose().Pos();

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
    node = std::make_unique<ros::NodeHandle>();
    node->setCallbackQueue(&queue);
    lineSub = node->subscribe("sim/line_length", 1, &Tether::SetLength, this);
    ctdPub = node->advertise<geometry_msgs::PointStamped>(
        "sim/ctd_position", 1);

    // Gazebo clients receive drum and line geometry over the visual topic
    transport.reset(new transport::Node());
    transport->Init(model->GetWorld()->Name());
    visualPub = transport->Advertise<msgs::Visual>("~/visual");
    update = event::Events::ConnectWorldUpdateBegin(
        std::bind(&Tether::Step, this, std::placeholders::_1));
  }

  private: void SetLength(const std_msgs::Float64ConstPtr &msg)
  {
    if (std::isfinite(msg->data) && msg->data >= 0)
      length = msg->data;
  }

  private: void Step(const common::UpdateInfo &info)
  {
    queue.callAvailable(ros::WallDuration(0));
    const auto pose = body->WorldPose();
    const auto liftingEye = pose.Pos() + pose.Rot().RotateVector(eye);
    const auto span = liftingEye - anchor;
    const double distance = span.Length();

    // Slack line exerts no force. Once taut, the line pulls along the span
    // and the damper opposes the lifting eye's speed along that direction.
    if (distance > length)
    {
      const auto direction = span / distance;
      const double speed = direction.Dot(body->WorldLinearVel(eye));
      const double tension = std::max(0.0, stiffness * (distance - length)
                                            + damping * speed);
      body->AddForceAtWorldPosition(-tension * direction, liftingEye);
    }

    // Approximate the submerged part of the upright 1.02 m housing
    const double submerged = std::clamp((0.51 - pose.Pos().Z()) / 1.02,
                                        0.0, 1.0);
    const auto velocity = body->WorldLinearVel() - current;
    const auto drag = -(18.0 + 32.0 * velocity.Length()) * velocity;
    const auto buoyancy = -displacedMass * body->GetWorld()->Gravity();
    body->AddForce(submerged * (buoyancy + drag));
    body->AddTorque(-5.0 * submerged * body->WorldAngularVel());

    // Sample the probe at 10 Hz and draw the line at 25 Hz in simulated time
    if (Due(info.simTime, lastSample, 0.1))
    {
      geometry_msgs::PointStamped sample;
      sample.header.stamp = ros::Time(info.simTime.sec, info.simTime.nsec);
      sample.header.frame_id = "world";
      const auto probe = pose.Pos() + pose.Rot().RotateVector(probeOffset);
      sample.point.x = probe.X();
      sample.point.y = probe.Y();
      sample.point.z = probe.Z();
      ctdPub.publish(sample);
      lastSample = info.simTime;
    }

    // The physics runs at full rate even when no viewer is connected
    if (!visualPub->HasConnections() || !Due(info.simTime, lastVisual, 0.04))
      return;
    lastVisual = info.simTime;
    if (!visualsReady && !FindVisuals())
      return;

    DrawLine(liftingEye, span, distance);
    const double angle = 2.0 * M_PI * (length - initialLength) / circumference;
    auto stripe = stripePose;
    stripe.Rot() = ignition::math::Quaterniond(0, angle, 0) * stripe.Rot();
    structure->SetVisualPose(stripeId, stripe);
  }

  // Publish immediately when Gazebo rewinds the clock
  private: static bool Due(const common::Time &now, const common::Time &last,
                           double period)
  {
    return now < last || now - last >= common::Time(period);
  }

  // Gazebo identifies the original SDF visuals by their server-assigned IDs
  private: bool FindVisuals()
  {
    const auto barge = body->GetWorld()->ModelByName("barge");
    if (!barge)
      return false;
    structure = barge->GetLink("structure");
    visualsReady = structure && structure->VisualId("upper_line", upperId)
                   && structure->VisualId("lower_line", lowerId)
                   && structure->VisualId("drum_stripe", stripeId)
                   && structure->VisualPose(stripeId, stripePose);
    return visualsReady;
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
    DrawSegment("upper_line", upperId, anchor, elbow);
    DrawSegment("lower_line", lowerId, elbow, end);
  }

  private: void DrawSegment(const std::string &name, uint32_t id,
                            const Vector3d &a, const Vector3d &b)
  {
    // A Gazebo cylinder is centered on its local Z axis
    const auto delta = b - a;
    const auto center = (a + b) * 0.5;
    const double pitch = std::atan2(std::hypot(delta.X(), delta.Y()),
                                    delta.Z());
    const double yaw = std::atan2(delta.Y(), delta.X());

    // Update the existing barge visual between the segment endpoints
    msgs::Visual visual;
    const auto parent = structure->GetScopedName();
    visual.set_name(parent + "::" + name);
    visual.set_id(id);
    visual.set_parent_name(parent);
    visual.set_parent_id(structure->GetId());
    msgs::Set(visual.mutable_pose(), ignition::math::Pose3d(
        center, ignition::math::Quaterniond(0, pitch, yaw)));
    auto geometry = visual.mutable_geometry();
    geometry->set_type(msgs::Geometry::CYLINDER);
    auto cylinder = geometry->mutable_cylinder();
    cylinder->set_radius(0.009);
    cylinder->set_length(std::max(delta.Length(), 1e-6));
    visualPub->Publish(visual);
  }

  private: physics::LinkPtr body, structure;
  private: bool visualsReady = false;
  private: uint32_t upperId, lowerId, stripeId;
  private: ignition::math::Pose3d stripePose;
  private: Vector3d anchor, current, eye, probeOffset;
  private: double circumference, damping, displacedMass, initialLength;
  private: double length, stiffness;
  private: common::Time lastSample, lastVisual;
  private: ros::CallbackQueue queue;
  private: std::unique_ptr<ros::NodeHandle> node;
  private: ros::Subscriber lineSub;
  private: ros::Publisher ctdPub;
  private: transport::NodePtr transport;
  private: transport::PublisherPtr visualPub;
  private: event::ConnectionPtr update;
};

GZ_REGISTER_MODEL_PLUGIN(Tether)
}
