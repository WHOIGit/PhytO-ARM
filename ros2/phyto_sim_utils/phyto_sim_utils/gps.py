from phyto_sim_utils.parameters import declare_number
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import NavSatFix, NavSatStatus


# Use the barge antenna in Gazebo or a stationary receiver on ROS time
def receive_fixes(node: Node, callback):
    topic = node.declare_parameter('fix_topic', '').value
    if topic:
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        return node.create_subscription(NavSatFix, topic, callback, qos)

    def coordinate(name, default):
        return float(declare_number(node, name, default))

    fix = NavSatFix(latitude=coordinate('latitude', 41.524),
                    longitude=coordinate('longitude', -70.672),
                    altitude=coordinate('altitude', 0.0))
    fix.header.frame_id = 'barge'
    fix.status.service = NavSatStatus.SERVICE_GPS

    def tick():
        fix.header.stamp = node.get_clock().now().to_msg()
        callback(fix)

    return node.create_timer(1.0, tick)
