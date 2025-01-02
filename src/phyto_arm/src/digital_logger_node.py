#!/usr/bin/env python3
import base64
import urllib.request

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

        self.auth = f'Basic {base64.b64encode(f"{self.username}:{self.password}".encode()).decode()}'

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
            # send a status request for each available outlet
            for outlet_index, _ in enumerate(self.outlets):
                # Construct request
                url = f'http://{self.address}/restapi/relay/outlets/{outlet_index}/state/'
                request = urllib.request.Request(url)
                request.add_header("Authorization", self.auth)

                try:
                    # Send the GET request
                    with urllib.request.urlopen(request) as response:
                        # Read and decode the response
                        response_data = response.read().decode('utf-8')
                except urllib.error.HTTPError as http_err:
                    raise ValueError(f"HTTP Error: {http_err.code} : {http_err.reason}") from http_err
                except urllib.error.URLError as url_err:
                    raise ValueError(f"URL Error: {url_err.reason}") from url_err

                # DL API uses 'true' and 'false' to denote outlet status, which need to be converted to Python bools
                if response_data == 'true':
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

        data = f'value={str(status).lower()}'
        url = f'http://{self.address}/restapi/relay/outlets/{outlet_num}/state/'

        # Create a PUT request
        req = urllib.request.Request(url, data=data.encode("utf-8"), method="PUT")
        req.add_header("Authorization", self.auth)
        req.add_header("X-CSRF", 'x')

        try:
            # Send the PUT request
            response = urllib.request.urlopen(req)
        except urllib.error.HTTPError as http_err:
            raise ValueError(f"HTTP Error: {http_err.code} : {http_err.reason}") from http_err
        except urllib.error.URLError as url_err:
            raise ValueError(f"URL Error: {url_err.reason}") from url_err

        result = response.read().decode('utf-8')

        rospy.loginfo(f'sent status={str(status).lower()} to {self.address}:{url}, received: code {response.status} : {result}')


if __name__ == '__main__':
    try:
        digital_logger = DigitalLogger()
        digital_logger.run()
    except rospy.ROSInterruptException:
        pass
