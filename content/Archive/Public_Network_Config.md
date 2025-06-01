---
title: "Network Configuration"
---

Networking a PhytO-ARM setup mostly involves having a network that can be accessed either from inside (downloading data while on -site) and outside (monitoring or troubleshooting remotely). Given the remote location of some of these sites and the need to have our own network available, we use a cellular modem that uses cellular signal and data to provide internet access to the platform.

#### System Overview

The modem/router device allows for network connections and for proper traffic direction. The SIM card allows the modem to access the cell network to gain internet access, which in turn means the fixed IP address of the SIM card can be used to remotely access network devices from beyond the network. All devices on the Local Area Network (LAN) can be accessed through public ports using the Wide Area Network (WAN). Entering the public IP address of the modem along with the specified port allows the modem to redirect the request to the appropriate device, as shown in the example diagram below.

![Alt](https://github.com/WHOIGit/PhytO-ARM/blob/master/website/static/images/Alma1.png)

The Sierra Wireless modem is the source of your connection to the entire system and makes remote monitoring possible.

#### Reserving IP addresses in Sierra Wireless ACE Manager

Here, we use setting up an IP reservation for a MAC Ethernet Module, though the steps here remain more or less the same for other devices.  This setup guide uses IP addresses specific to the modem and SIM card used in this deployment. Please modify the IP address as needed to match the SIM card in use.

1. Look up MAC address for the device. On the MAC module, this will be listed on a sticker at the bottom edge.

2. Log in to Sierra Wireless ACE manager and select the “LAN” tab, and expand the 3rd list titled DHCP Reservation List

![Alt](https://github.com/WHOIGit/PhytO-ARM/blob/master/website/static/images/MAC1.png)

3. Create a new entry and enter the MAC address for the module. If the motor is the only one on the network, assign it to 192.xxx.xx.105. If there is more than 1 motor on the network, increase the IP address by 1, so 192.xxx.xx.106, 192.xxx.xx.107, etc. 

![Alt](https://github.com/WHOIGit/PhytO-ARM/blob/master/website/static/images/MAC2.png)

4. If the device needs to be accessed remotely, select the “Security” tab and check the port forwarding table. When registering the module, the onboard computer it is connected to will be the point of access. For any computer, add two ports: one for remote desktop (Port 3389) and one for SSH (Port 22). 

![Alt](https://github.com/WHOIGit/PhytO-ARM/blob/master/website/static/images/MAC3.png)

  * Here, the public start port for remote access is 3932, the private port is 3389 (default Remote Desktop port)
  * The public start port for SSH is 3922, the private port is 20 (the default SSH port)

5. The modem will need to reboot for these changes to take effect. Be sure you have saved these changes and click the "Reboot" button in the top right. Often when devices have the wrong address, don't respond to pings, or otherwise don't seem to respond, it is because a setting was changed but without rebooting.

