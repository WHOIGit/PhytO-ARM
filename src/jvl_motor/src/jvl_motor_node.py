#!/usr/bin/env python3
import functools
import threading
import time

import rospy

import mac400
import jvl_motor.msg as msg
import jvl_motor.srv as srv

from pyModbusTCP.client import ModbusClient


WATCH_REGISTERS = [
    mac400.P_IST,
    mac400.V_IST_16,
    mac400.FLWERR,
    mac400.U_24V,
    mac400.ERR_STAT,
    mac400.U_BUS,
]


def read_registers(client, registers):
    min_addr, max_addr = \
        min(r.addr[0] for r in registers), \
        max(r.addr[-1] for r in registers)

    count = max_addr - min_addr + 1
    if count >= 125:  # max per read
        raise ValueError('Register range is too wide')

    with client.lock:
        values = client.read_holding_registers(min_addr, count)
    timestamp = rospy.Time.now()

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
        min_addr = list(reg_values.keys())[0].addr[0]
        success = client.write_multiple_registers(min_addr, encoded)

    if not success:
        raise RuntimeError('Failed to write registers')


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
    except RuntimeError:
        rospy.logerr('Failed to command motor movement')
    return srv.SetPositionCmdResponse()


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
    except RuntimeError:
        rospy.logerr('Failed to command motor movement')
    return srv.SetVelocityCmdResponse()


def cmd_set_position_envelope(client, request):
    try:
        write_registers(client, {
            mac400.MIN_P_IST:  request.min
        })
        write_registers(client, {
            mac400.MAX_P_IST:  request.max
        })
    except RuntimeError:
        rospy.logerr('Failed to set position envelope')
    return srv.SetPositionEnvelopeCmdResponse()


def cmd_stop(client, request):
    # Try continually to stop the motor, this is important
    while True:
        try:
            write_registers(client, {
                mac400.MODE_REG:  mac400.MODE.PASSIVE
            })
            break
        except RuntimeError:
            rospy.logerr('Failed to stop motor!!')
            time.sleep(0.1)

    return srv.StopCmdResponse()


def main():
    # Initialize this node
    rospy.init_node('jvl_motor', anonymous=True, log_level=rospy.DEBUG)

    # Connect to the Modbus server
    client = ModbusClient(
        host=rospy.get_param('~address'),
        port=rospy.get_param('~port', 502),
        auto_open=True  # keep us connected
    )
    client.lock = threading.Lock()

    # Create publishers for various message types
    electrical_pub = rospy.Publisher(
        '~electrical',
        msg.Electrical,
        queue_size=1
    )
    error_pub = rospy.Publisher(
        '~error',
        msg.Error,
        queue_size=1
    )
    motion_pub = rospy.Publisher(
        '~motion',
        msg.Motion,
        queue_size=1
    )
    
    # Create services
    services = [
        rospy.Service(
            '~set_position',
            srv.SetPositionCmd,
            functools.partial(cmd_set_position, client)
        ),
        rospy.Service(
            '~set_position_envelope',
            srv.SetPositionEnvelopeCmd,
            functools.partial(cmd_set_position_envelope, client)
        ),
        rospy.Service(
            '~set_velocity',
            srv.SetVelocityCmd,
            functools.partial(cmd_set_velocity, client)
        ),
        rospy.Service(
            '~stop',
            srv.StopCmd,
            functools.partial(cmd_stop, client)
        ),
    ]

    # We only send our error status if one of the bits has changed
    last_error = None


    rate = rospy.Rate(rospy.get_param('~refresh_rate', 1))  # hz
    while not rospy.is_shutdown():
        # The range of register addresses is too wide, so we need to do this in
        # two calls.
        try:
            reg_values, timestamp1 = read_registers(client, [
                mac400.MODE_REG,
                mac400.P_IST,
                mac400.V_IST_16,
                mac400.FLWERR,
                mac400.U_24V,
                mac400.ERR_STAT,
            ])
            reg_values2, timestamp2 = read_registers(client, [
                mac400.U_BUS,
            ])
            reg_values.update(reg_values2)
        except RuntimeError:
            rospy.logerr('Failed to read registers')
            rate.sleep()
            continue

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
        last_error = err_stat

        rate.sleep()

    for service in services:
        service.shutdown('node shutting down')


if __name__ == '__main__':
    main()
