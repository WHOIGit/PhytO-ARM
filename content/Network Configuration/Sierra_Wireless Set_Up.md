---
title: "Sierra Wireless Ace Manager"
---

#### Setting up the module in Sierra Wireless ACE Manager

1.  Look up MAC address for motor module, this will be listed on a sticker at the bottom edge

![Alt](https://github.com/WHOIGit/PhytO-ARM/blob/master/website/static/images/JVL1.jpg)

2.  Log in to Sierra Wireless ACE manager (this is done by typing "<https://> your IP address : your port number " into your browser) and select the "LAN" tab, and expand the 3rd list titled DHCP Reservation List

![Alt](https://github.com/WHOIGit/PhytO-ARM/blob/master/website/static/images/MAC1.png)

3.  Create a new entry and enter the MAC address for the module. If the motor is the only one on the network, assign it to 192.168.13.3. We label all of our JVL motors as .3 for simplicity.

![Alt](https://github.com/WHOIGit/PhytO-ARM/blob/master/website/static/images/MAC2.png)

4.  To set up remote access directly to the Win 10 NUC, select the "Security" tab and check the port forwarding table. There should be two entries for Win 10 NUC (IP address 192.168.13.2), one for remote desktop and one for SSH. If these aren't there, they need to be added.

![Alt](https://github.com/WHOIGit/PhytO-ARM/blob/master/website/static/images/MAC3.png)

-   The public start port for remote access will be 3932, the private port will be 3389 (default Remote Desktop port)
-   The public start port for SSH will be 3922, the private port will be 20 (the default SSH port)
-   The motor does not have any port forwarding set up as it should never need to be remotely accessed, all access will be through the Win 10 machine at 192.168.13.2

5.  The module does not need to be connected via Ethernet for reserving the address in ACE Manager, however you do need the physical network set up for the next steps. Be sure to connect all the components via Ethernet to a switch

-   Ensure the Win10 machine, motor, and modem are all plugged into the same switch OR
-   All components are plugged in any switch that has been daisy chained to another
-   There are many components that all require power: the modem, switch, Win10 NUC, adjustable DC power supply for the motor control circuit, and AC power for the motor drive circuit, plus any additional computers or monitors.
