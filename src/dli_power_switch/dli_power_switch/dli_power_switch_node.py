#!/usr/bin/env python3
"""
Functionality for monitoring and controlling a Digital Loggers Web Power Switch Pro model.
"""
import functools
import re

import rclpy

from typing import Dict

from dli_power_switch.api import DLIClient
from dli_power_switch_msgs.msg import OutletStatus
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.publisher import Publisher
from std_msgs.msg import Bool


def control_outlet(node: Node, client: DLIClient, number: int, msg: Bool) -> None:
    """
    Send the given msg to the digital logger as an HTTP request.
    """
    client.set_outlet_state(number - 1, msg.data)
    node.get_logger().info(f'Switched {"ON" if msg.data else "OFF"} outlet {number}')


def monitor_outlets(node: Node) -> None:
    """
    Run the Digital Logger power switch node. Publishes outlet statuses at
    ~/outlet/{name}/status. The outlets can be controlled by publishing a
    std_msgs/Bool message to ~/outlet/{name}/control.
    """
    node.declare_parameters('', [
        ('address', Parameter.Type.STRING),
        ('password', Parameter.Type.STRING),
        ('username', Parameter.Type.STRING),
    ])
    # Each configured outlet has an indexed name and outlet number.
    for name in node._parameter_overrides:
        match = re.fullmatch(r'outlets\.[0-9]+\.(name|outlet)', name)
        if match:
            kind = Parameter.Type.STRING if match[1] == 'name' else Parameter.Type.INTEGER
            node.declare_parameter(name, kind)
    settings = node.get_parameters_by_prefix('outlets')
    if not settings:
        raise KeyError('outlets')
    indices = {int(key.split('.')[0]) for key in settings}
    outlets = [(settings[f'{i}.name'].value, settings[f'{i}.outlet'].value)
               for i in range(len(indices))]

    client = DLIClient(
        node.get_parameter('address').value,
        node.get_parameter('username').value,
        node.get_parameter('password').value
    )

    # Set up publisher and subscriber for each outlet
    outlet_pubs: Dict[int, Publisher] = {}
    for name, num in outlets:
        outlet_pubs[num] = node.create_publisher(
            OutletStatus, f'~/outlet/{name}/status', 10
        )
        node.create_subscription(
            Bool, f'~/outlet/{name}/control',
            functools.partial(control_outlet, node, client, num), 10,
            callback_group=MutuallyExclusiveCallbackGroup()
        )

    # Monitor outlets at 1Hz
    def poll_outlets():
        # send a status request for each configured outlet
        for _, num in outlets:
            outlet_status = OutletStatus()
            outlet_status.is_active = client.get_outlet_state(num - 1)
            outlet_status.header.stamp = outlet_status.ds_header.io_time = \
                node.get_clock().now().to_msg()
            outlet_pubs[num].publish(outlet_status)

    # Poll immediately even if the simulation clock is paused at startup.
    node.create_guard_condition(poll_outlets).trigger()
    node.create_timer(1.0, poll_outlets)


def run_dli_power_switch_node() -> None:
    rclpy.init()
    node = rclpy.create_node('dli_power_switch')
    executor = MultiThreadedExecutor(num_threads=2)
    try:
        monitor_outlets(node)
        rclpy.spin(node, executor=executor)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    run_dli_power_switch_node()
