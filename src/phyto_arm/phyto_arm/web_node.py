#!/usr/bin/env python3
import functools
import os
import threading

import rclpy

from aiohttp import web
from phyto_sim_utils.parameters import declare_matching_parameters
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rosidl_runtime_py.utilities import get_message


response_values = {}


def capture_field_value(node, name, field, msg):
    if not hasattr(msg, field):
        node.get_logger().error(f'Field {field} not found in msg type {type(msg)}. Check config {name}.')
        return
    response_values[name] = getattr(msg, field)


app = web.Application()
routes = web.RouteTableDef()


@routes.get('/')
async def index(request):
    return web.json_response(response_values)


app.add_routes(routes)


def configure_node(node: Node) -> None:
    declare_matching_parameters(node, r'field_map\.[^.]+\.(environment|topic|topic_field)',
                                Parameter.Type.STRING)
    declare_matching_parameters(node, r'field_map\.[^.]+\.default(\..+)?')

    # The field map can contain nested JSON defaults, including indexed lists.
    def read_setting(prefix):
        if node.has_parameter(prefix):
            return node.get_parameter(prefix).value
        names = dict.fromkeys(key.split('.')[0] for key in node.get_parameters_by_prefix(prefix))
        if names and names.keys() == {str(i) for i in range(len(names))}:
            return [read_setting(f'{prefix}.{i}') for i in range(len(names))]
        return {name: read_setting(f'{prefix}.{name}') for name in names}

    # Read config, set defaults, create subscribers
    field_map = read_setting('field_map')
    if not field_map:
        raise KeyError('field_map')
    pending = {}
    for name, config in field_map.items():
        if 'environment' in config:
            if 'topic' in config:
                node.get_logger().error(f'Config {name} invalid. Mixing topic and environment not supported')
            var_name = config['environment']
            if var_name in os.environ:
                response_values[name] = os.environ[var_name]
            elif 'default' in config:
                node.get_logger().warning(f'Env var {var_name} not found, switching to default for config {name}')
                response_values[name] = config['default']
            else:
                node.get_logger().error(f'Config {name} invalid. Env var {var_name} not found and no default provided')
        elif 'default' in config:
            response_values[name] = config['default']
            # Subscriber optional; skip if no topic. Makes default permanent
            if 'topic' in config:
                if 'topic_field' not in config:
                    node.get_logger().error(f'Config {name} invalid, topic_field required when topic used')
                pending[name] = (config['topic'], config['topic_field'])
        else:
            node.get_logger().error(f'Config {name} invalid, must have "default" or "environment" set')

    # Discover each configured topic's actual type and match its publishers' QoS.
    def discover():
        for name, (topic, field) in list(pending.items()):
            publishers = node.get_publishers_info_by_topic(topic)
            if not publishers:
                continue
            qos = QoSProfile(depth=10)
            if all(p.qos_profile.durability == DurabilityPolicy.TRANSIENT_LOCAL for p in publishers):
                qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
            if any(p.qos_profile.reliability == ReliabilityPolicy.BEST_EFFORT for p in publishers):
                qos.reliability = ReliabilityPolicy.BEST_EFFORT
            node.create_subscription(get_message(publishers[0].topic_type), topic,
                                     functools.partial(capture_field_value, node, name, field), qos)
            del pending[name]
        if not pending:
            timer.cancel()

    timer = node.create_timer(0.1, discover)
    discover()


def main():
    rclpy.init()
    node = rclpy.create_node('web_node')
    executor = SingleThreadedExecutor()
    try:
        configure_node(node)
        executor.add_node(node)
        dispatch = threading.Thread(target=executor.spin)
        dispatch.start()
        try:
            web.run_app(app, port=8098)
        finally:
            executor.shutdown()
            dispatch.join()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
