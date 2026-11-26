from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rosidl_runtime_py.utilities import get_message


def subscribe_sensor(node, topic, callback, msg_type=None, *, callback_group=None):
    publishers = node.get_publishers_info_by_topic(topic)
    if not publishers:
        return None
    qos = QoSProfile(depth=10)
    if all(p.qos_profile.durability == DurabilityPolicy.TRANSIENT_LOCAL for p in publishers):
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
    if any(p.qos_profile.reliability == ReliabilityPolicy.BEST_EFFORT for p in publishers):
        qos.reliability = ReliabilityPolicy.BEST_EFFORT
    return node.create_subscription(msg_type or get_message(publishers[0].topic_type), topic,
                                    callback, qos, callback_group=callback_group)
