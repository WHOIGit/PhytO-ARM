#!/usr/bin/env python3
import functools
import json

import rospy

import ifcb.srv as srv

from ifcbclient import IFCBClient

from ds_core_msgs.msg import RawData


# Apologies for the horrible naming. This is the ROS service handler that
# invokes send_command() and returns an empty response.
def do_command(client, pub, command):
    send_command(client, pub, command.command)
    return srv.CommandResponse()


# Send a command to the IFCB host and publish it to ROS
def send_command(client, pub, command):
    # Construct the message we are going to send
    msg = RawData()
    msg.header.stamp = msg.ds_header.io_time = rospy.Time.now()
    msg.data_direction = RawData.DATA_OUT
    msg.data = command.encode()

    # Send the command
    client.relay_message_to_host(command)

    # Publish the message after sending it, so that the timestamp is more
    # accurate.
    pub.publish(msg)


# Publish incoming "raw" messages from the IFCB as ROS messages
def on_any_message(pub, data):
    sender_id, smsgsrc, seqno, data = data

    msg = RawData()
    msg.header.seq = seqno
    msg.header.stamp = msg.ds_header.io_time = rospy.Time.now()
    msg.data_direction = RawData.DATA_IN
    msg.data = data.encode()
    pub.publish(msg)


def main():
    rospy.init_node('ifcb', anonymous=True)

    # Publishers for raw messages coming in and out of the IFCB
    rx_pub = rospy.Publisher('/ifcb/in',  RawData, queue_size=5)
    tx_pub = rospy.Publisher('/ifcb/out', RawData, queue_size=5)

    # Create an IFCB websocket API client
    client = IFCBClient(
        f'ws://{rospy.get_param("/ifcb/address")}'\
            f':{rospy.get_param("/ifcb/port", 8092)}/ifcbHub',
        rospy.get_param('/ifcb/serial'),
    )

    # Publish all messages that come in, prior to being parsed.
    # Note: This relies on internal implementation details of pyifcbclient.
    client.hub_connection.on('messageRelayed',
        functools.partial(on_any_message, rx_pub))
    
    # Create a ROS service for sending commands
    rospy.Service(
        f'{rospy.get_name()}/command',
        srv.Command,
        functools.partial(do_command, client, tx_pub),
    )

    # Connect the client and let other threads do their thing
    client.connect()
    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
