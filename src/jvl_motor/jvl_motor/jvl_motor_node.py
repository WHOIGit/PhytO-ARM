#!/usr/bin/env python3
import contextlib
import functools
import os
import random
import threading
import time

import rclpy

from jvl_motor import mac400
from jvl_motor_msgs import msg, srv
from phyto_sim_utils.parameters import declare_number
from pyModbusTCP.client import ModbusClient
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.logging import LoggingSeverity
from rclpy.node import Node
from rclpy.parameter import Parameter


WATCH_REGISTERS = [
    mac400.P_IST,
    mac400.V_IST_16,
    mac400.FLWERR,
    mac400.U_24V,
    mac400.ERR_STAT,
    mac400.U_BUS,
]


def read_registers(node, client, registers):
    min_addr, max_addr = \
        min(r.addr[0] for r in registers), \
        max(r.addr[-1] for r in registers)

    count = max_addr - min_addr + 1
    if count >= 125:  # max per read
        raise ValueError('Register range is too wide')

    with client.lock:
        values = client.read_holding_registers(min_addr, count)
    timestamp = node.get_clock().now().to_msg()

    if values is None:
        raise RuntimeError('Failed to read registers')

    # Return a mapping of register -> decoded value, and the timestamp
    output = {}
    for reg in registers:
        output[reg] = reg.decode(*[ values[a - min_addr] for a in reg.addr ])
    return output, timestamp


def write_registers(client, reg_values):
    for r, rnext in zip(reg_values.keys(), list(reg_values.keys())[1:]):
        if rnext.addr[0] != r.addr[-1] + 1:
            raise ValueError('Cannot write non-contiguous registers')

    encoded = []
    for r, v in reg_values.items():
        encoded.extend(r.encode(v))

    with client.lock:
        if not client.accept_commands:
            if reg_values.get(mac400.MODE_REG) != mac400.MODE.PASSIVE:
                raise RuntimeError('Motor driver is shutting down')
        min_addr = list(reg_values.keys())[0].addr[0]
        success = client.write_multiple_registers(min_addr, encoded)

    if not success:
        raise RuntimeError('Failed to write registers')

def cmd_zero_position(client, request):
    try:
        write_registers(client, {
            mac400.P_IST:  0
        })
    except RuntimeError as err:
        raise RuntimeError('Failed to zero motor position') from err

def cmd_set_position(client, request):
    try:
        write_registers(client, {
            mac400.MODE_REG:  mac400.MODE.POSITION,
            mac400.P_SOLL:    request.position,
            mac400.P_NEW:     0,
            mac400.V_SOLL:    request.velocity,
            mac400.A_SOLL:    request.acceleration,
            mac400.T_SOLL:    request.torque,
        })
    except RuntimeError as err:
        raise RuntimeError('Failed to command motor movement') from err


def cmd_set_velocity(client, request):
    try:
        write_registers(client, {
            mac400.MODE_REG:  mac400.MODE.VELOCITY,
            mac400.P_SOLL:    0,
            mac400.P_NEW:     0,
            mac400.V_SOLL:    request.velocity,
            mac400.A_SOLL:    request.acceleration,
            mac400.T_SOLL:    request.torque,
        })
    except RuntimeError as err:
        raise RuntimeError('Failed to set motor velocity') from err


def cmd_set_position_envelope(client, request):
    try:
        write_registers(client, {
            mac400.MIN_P_IST:  request.min
        })
        write_registers(client, {
            mac400.MAX_P_IST:  request.max
        })
    except RuntimeError as err:
        raise RuntimeError('Failed to set position envelope') from err


def cmd_stop(node, client, request):
    # Try continually to stop the motor, this is important
    while True:
        try:
            write_registers(client, {
                mac400.MODE_REG:  mac400.MODE.PASSIVE
            })
            break
        except RuntimeError:
            node.get_logger().error('Failed to stop motor!!')
            time.sleep(0.1)



# Publish the latest motor readings and return the last known error status
def publish_status(node, client, electrical_pub, error_pub, motion_pub, last_error):
    # The range of register addresses is too wide, so we need to do this in
    # two calls.
    try:
        reg_values, timestamp1 = read_registers(node, client, [
            mac400.MODE_REG,
            mac400.P_IST,
            mac400.V_IST_16,
            mac400.FLWERR,
            mac400.U_24V,
            mac400.ERR_STAT,
        ])
        reg_values2, timestamp2 = read_registers(node, client, [
            mac400.U_BUS,
        ])
        reg_values.update(reg_values2)
    except RuntimeError:
        node.get_logger().error('Failed to read registers, retrying on loop')
        return last_error

    # Publish information about the electrical state
    m = msg.Electrical()
    m.header.stamp = m.ds_header.io_time = timestamp1
    m.u_24v = reg_values[mac400.U_24V]
    m.u_bus = reg_values[mac400.U_BUS]
    electrical_pub.publish(m)

    # Publish information about motion
    m = msg.Motion()
    m.header.stamp = m.ds_header.io_time = timestamp1
    m.mode = reg_values[mac400.MODE_REG].value
    m.position = reg_values[mac400.P_IST]
    m.velocity = reg_values[mac400.V_IST_16]
    # TODO: FLWERR
    motion_pub.publish(m)

    # If the error state has changed, publish what changed. For now,
    # re-pack bits into a 32-bit integer.
    err_stat = mac400.implode_bits(reg_values[mac400.ERR_STAT])
    if last_error is not None and last_error != err_stat:
        m = msg.Error()
        m.header.stamp = m.ds_header.io_time = timestamp1
        m.err_stat = err_stat
        m.change_mask = err_stat ^ last_error
        error_pub.publish(m)
    return err_stat


def configure_node(node: Node, cleanup: contextlib.ExitStack) -> None:
    node.get_logger().set_level(LoggingSeverity.DEBUG)

    node.declare_parameters('', [('address', Parameter.Type.STRING), ('port', 502)])

    # Connect to the Modbus server
    client = ModbusClient(
        host=node.get_parameter('address').value,
        port=node.get_parameter('port').value,
        auto_open=True  # keep us connected
    )
    client.lock = threading.Lock()
    client.accept_commands = True

    def stop_on_exit():
        # An in-flight write finishes under the lock before this final stop
        client.accept_commands = False
        try:
            write_registers(client, {mac400.MODE_REG: mac400.MODE.PASSIVE})
        except RuntimeError as err:
            node.get_logger().error(
                f'Motor did not acknowledge PASSIVE during shutdown: {err}')

    if node.declare_parameter('stop_on_exit', True).value:
        cleanup.callback(stop_on_exit)

    # Create publishers for various message types
    electrical_pub = node.create_publisher(msg.Electrical, '~/electrical', 1)
    error_pub = node.create_publisher(msg.Error, '~/error', 1)
    motion_pub = node.create_publisher(msg.Motion, '~/motion', 1)

    # Create services
    def create_service(kind, name, handler):
        def handle(request, response):
            try:
                handler(client, request)
                response.success = True
            except Exception as err:
                node.get_logger().error(f"Service handler for '{name}' raised: {err}")
                response.message = str(err)
            return response

        node.create_service(kind, name, handle, callback_group=MutuallyExclusiveCallbackGroup())

    create_service(srv.ZeroCmd, '~/zero_position', cmd_zero_position)
    create_service(srv.SetPositionCmd, '~/set_position', cmd_set_position)
    create_service(srv.SetPositionEnvelopeCmd, '~/set_position_envelope', cmd_set_position_envelope)
    create_service(srv.SetVelocityCmd, '~/set_velocity', cmd_set_velocity)
    create_service(srv.StopCmd, '~/stop', functools.partial(cmd_stop, node))

    # We only send our error status if one of the bits has changed
    last_error = None

    def poll():
        nonlocal last_error
        last_error = publish_status(
            node, client, electrical_pub, error_pub, motion_pub, last_error
        )

    # Poll immediately and keep clock and service callbacks available during I/O.
    refresh_rate = declare_number(node, 'refresh_rate', 1)
    polling = MutuallyExclusiveCallbackGroup()
    node.create_guard_condition(poll, callback_group=polling).trigger()
    node.create_timer(1 / refresh_rate, poll, callback_group=polling)


def main():
    # Initialize this node
    rclpy.init()
    node = rclpy.create_node(f'jvl_motor_{os.getpid()}_{random.getrandbits(32):08x}')
    executor = MultiThreadedExecutor()
    try:
        with contextlib.ExitStack() as cleanup:
            configure_node(node, cleanup)
            rclpy.spin(node, executor=executor)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
