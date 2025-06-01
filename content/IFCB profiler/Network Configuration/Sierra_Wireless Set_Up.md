---
title: "Sierra Wireless Ace Manager"
weight: 8
---

#### Setting up the module in Sierra Wireless ACE Manager

1.  Look up MAC address for motor module, this will be listed on a sticker at the bottom edge

![Alt](/images/JVLMAC.jpeg)

2.  Log in to Sierra Wireless ACE manager (this is done by typing "<https://> your IP address : your port number " into your browser) and entering the unsername and password assigned to your sierra modem. Then select the "LAN" tab, and expand the 3rd list titled DHCP Reservation List

![Alt](/images/MAC1.png)

3.  Create a new entry and enter the MAC address for the module. If the motor is the only one on the network, assign it to 192.168.13.3. (IP address 192.168.13.xxx is the standard assigned to all Sierra Wireless MP70s. Ending digits larger than .100 are unreserved and automatically assigned. We advice assigning all components of the system IP addresses that end with values less than .100 and maintaining a standard naming convention between Phyto-Arm Systems.) We label all of our JVL motors as .3 for simplicity. You may also have to do this same process for the WIN10 machine. Alternatively, we have been using a IFCB as the on-board computer, in this case follow all steps the same simply replace any WIN10 NUC reserving  and access with IFCB machine.

![Alt](/images/MAC2.png)

4.  To set up remote access directly to the Win 10 NUC, select the "Security" tab and check the port forwarding table. There should be two entries for Win 10 NUC (IP address 192.168.13.2), one for remote desktop and one for SSH. If these aren't there, they need to be added.

![Alt](/images/MAC3.png)

-   The public start port for remote access will be 3932, the private port will be 3389 (default Remote Desktop port)
-   The public start port for SSH will be 3922, the private port will be 20 (the default SSH port)
-   The motor does not have any port forwarding set up as it should never need to be remotely accessed, all access will be through the Win 10 machine at 192.168.13.2

5.  The module does not need to be connected via Ethernet for reserving the address in ACE Manager, however you do need the physical network set up for the next steps of MacTalk setup and usage. Be sure to connect all the components via Ethernet to a switch.
