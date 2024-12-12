#!/usr/bin/env python3
import base64
import http.client

import rospy

from phyto_arm.msg import OutletStatus


class DigitalLogger:
    def __init__(self):
        rospy.init_node('digital_logger')

        # subscribe to the digital logger control topic
        rospy.Subscriber('/digital_logger/control', OutletStatus, self.control_outlet)

        self.username = rospy.get_param('~username')
        self.password = rospy.get_param('~password')
        self.address = rospy.get_param('~address')
        self.outlets = rospy.get_param('~outlets')
        self.outlet_names = {outlet['name']: int(outlet['outlet']) for outlet in self.outlets}

        # use separate connections to prevent processes from accessing the same connection simultaneously
        self.get_conn = http.client.HTTPConnection(self.address)
        self.put_conn = http.client.HTTPConnection(self.address)
        auth_string = f"{self.username}:{self.password}"
        auth_bytes = auth_string.encode('utf-8')
        encoded_auth = base64.b64encode(auth_bytes).decode('utf-8')
        self.auth = f"Basic {encoded_auth}"

        self.outlet_publishers = []
        for outlet_num, _ in enumerate(self.outlets):
            self.outlet_publishers.append(rospy.Publisher(f'/digital_logger/outlet/{outlet_num}/status/', OutletStatus, queue_size=10))

        # Monitor outlets at 1Hz
        self.rate = rospy.Rate(1)

    def run(self):
        """
        Run the digital logger node. Publishes outlet statuses at 
        /digital_logger/outlets/{outlet num}/status. The outlets can be controlled by publishing a 
        OutletStatus message to /digital_logger/control. 
        """
        while not rospy.is_shutdown():
            header = {
                'Content-Type': 'application/json',
                'Authorization': self.auth,
            }

            # send a status request for each available outlet
            for outlet_index, _ in enumerate(self.outlets):
                path = f'/restapi/relay/outlets/{outlet_index}/state/'

                self.get_conn.request('GET', path, headers=header)
                response = self.get_conn.getresponse()

                # Check if the request was successful
                if response.status != 200:
                    raise RuntimeError(f'Failed to get outlet status from digital logger. Response: {response.status} {response.reason}')

                outlet_result = response.read().decode('utf-8')

                # DL API uses 'true' and 'false' to denote outlet status, which need to be converted to Python bools
                if outlet_result == 'true':
                    status = 'on'
                else:
                    status = 'off'

                # publish the status of each outlet to its specific topic
                outlet_status = OutletStatus()
                outlet_status.name = self.outlets[outlet_index]['name']
                outlet_status.status = status

                self.outlet_publishers[outlet_index].publish(outlet_status)

            self.rate.sleep()

    def control_outlet(self, msg):
        """
        Send the given msg to the digital logger as an HTTP request.
        """

        outlet_num = self.outlet_names.get(msg.name)

        status = msg.status == 'on'

        # Define header
        header = {
                'X-CSRF': 'x',
                'Content-Type': 'application/x-www-form-urlencoded',
                'Accept': 'application/json',
                'Authorization': self.auth,
        }

        data = f'value={str(status).lower()}'
        path = f'/restapi/relay/outlets/{outlet_num}/state/'

        self.put_conn.request('PUT', path, data.encode(), header)
        response = self.put_conn.getresponse()

        # Check if the request was successful
        if response.status != 204:
            raise RuntimeError(f'Failed to set outlet status of digital logger. Response: {response.status} {response.reason}')

        result = response.read().decode('utf-8')

        rospy.loginfo(f'sent status={str(status).lower()} to {self.address}:{path}, received: code {response.status} : {result}')


if __name__ == '__main__':
    try:
        digital_logger = DigitalLogger()
        digital_logger.run()
    except rospy.ROSInterruptException:
        pass
