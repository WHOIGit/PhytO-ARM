#!/usr/bin/env python3

import rospy
import std_msgs.msg

import mac400registers

from pyModbusTCP.client import ModbusClient


def read_registers(client, registers):
    '''
    Efficiently requests values for a set of registers, attempting to minimize
    the number of requests and the time difference between requests.

    Returns a dictionary of Register -> (decoded value, timestamp)
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
            decoded = reg.decode(*[ values[a - grp[0].addr[0]]
                                    for a in reg.addr ])
            output[reg] = (decoded, timestamp)
    return output


def main():
    # Look up the requested registers
    registers = [ mac400registers.register_for_name(reg) \
                  for reg in rospy.get_param('/jvl_motor/registers') ]

    # Create a publisher for each register we are monitoring
    publishers = {}
    for reg in registers:
        publishers[reg] = rospy.Publisher(f'/jvl/register/{reg.name}',
                                          std_msgs.msg.Int32,
                                          queue_size=10)

    # Initialize this node
    rospy.init_node('jvl_motor', anonymous=True)

    # Connect to the Modbus server
    client = ModbusClient(
        host=rospy.get_param('/jvl_motor/address'),
        port=rospy.get_param('/jvl_motor/port', 502),
        auto_open=True  # keep us connected
    )

    rate = rospy.Rate(rospy.get_param('/jvl_motor/refresh_rate', 1))  # hz
    while not rospy.is_shutdown():
        reg_values = read_registers(client, registers)
        for reg, (value, timestamp) in reg_values.items():
            publishers[reg].publish(value)
        rate.sleep()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
