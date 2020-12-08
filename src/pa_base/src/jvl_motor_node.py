#!/usr/bin/env python3
import functools

import rospy

import mac400
import pa_base.srv as srv

from pyModbusTCP.client import ModbusClient

from pa_base.msg import JVLMotorRegisterRaw


WATCH_REGISTERS = [
    mac400.P_IST,
    mac400.V_IST_16,
    mac400.FLWERR,
    mac400.U_24V,
    mac400.ERR_STAT,
    mac400.U_BUS,
]


def read_registers(client, registers):
    '''
    Efficiently requests values for a set of registers, attempting to minimize
    the number of requests and the time difference between requests.

    Returns a dictionary of Register -> (raw values, timestamp)
    '''
    # Group registers into contiguous ranges that can be read at one time
    registers = sorted(registers, key=lambda r: r.addr[0])
    groups = [ [ registers[0] ] ]
    for reg in registers[1:]:
        if reg.addr[-1] - groups[-1][0].addr[0] >= 125:  # max per read command
            groups.append([])
        groups[-1].append(reg)

    # Read registers from the server and timestamp them
    valueses, timestamps = [], []
    for grp in groups:
        count = grp[-1].addr[-1] - grp[0].addr[0] + 1
        valueses.append(client.read_holding_registers(grp[0].addr[0], count))
        timestamps.append(rospy.Time.now())

    # Perform deferred decoding and arrange values for output
    output = {}
    for grp, values, timestamp in zip(groups, valueses, timestamps):
        for reg in grp:
            these_values = [ values[a - grp[0].addr[0]] for a in reg.addr ]
            output[reg] = (these_values, timestamp)
    return output


def cmd_move(client, request):
    rospy.loginfo('Moving motor')
    client.write_multiple_registers(
        mac400.MODE_REG.addr[0],
        [
            *mac400.MODE_REG.encode(mac400.MODE.VELOCITY),
            *mac400.P_SOLL.encode(0),
            *mac400.P_NEW.encode(0),
            *mac400.V_SOLL.encode(request.velocity),
            *mac400.A_SOLL.encode(request.acceleration),
            *mac400.T_SOLL.encode(request.torque),
        ]
    )
    return srv.JVLMotorMoveCmdResponse()


def cmd_stop(client, request):
    rospy.loginfo('Stopping motor')
    client.write_multiple_registers(
        mac400.MODE_REG.addr[0],
        mac400.MODE_REG.encode(mac400.MODE.PASSIVE),
    )
    return srv.JVLMotorStopCmdResponse()



def main():
    # Connect to the Modbus server
    client = ModbusClient(
        host=rospy.get_param('/jvl_motor/address'),
        port=rospy.get_param('/jvl_motor/port', 502),
        auto_open=True  # keep us connected
    )

    # Initialize this node
    rospy.init_node('jvl_motor', anonymous=True)

    # Create a publisher for each register we are monitoring
    publishers = {}
    for reg in WATCH_REGISTERS:
        publishers[reg] = rospy.Publisher(
            f'{rospy.get_name()}/register/{reg.name}',
            JVLMotorRegisterRaw,
            queue_size=1
        )
    
    # Create services for start and stop
    services = [
        rospy.Service(
            f'{rospy.get_name()}/move',
            srv.JVLMotorMoveCmd,
            functools.partial(cmd_move, client)
        ),
        rospy.Service(
            f'{rospy.get_name()}/stop',
            srv.JVLMotorStopCmd,
            functools.partial(cmd_stop, client)
        ),
    ]

    rate = rospy.Rate(rospy.get_param('/jvl_motor/refresh_rate', 1))  # hz
    while not rospy.is_shutdown():
        reg_values = read_registers(client, WATCH_REGISTERS)
        for reg, (values, timestamp) in reg_values.items():
            msg = JVLMotorRegisterRaw()
            msg.header.stamp = msg.ds_header.io_time = timestamp
            msg.raw_values = values

            publishers[reg].publish(msg)
        rate.sleep()

    for service in services:
        service.shutdown('node shutting down')


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
