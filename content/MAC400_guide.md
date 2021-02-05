# MAC 400 Motor Guide

This guide serves as a reference manual for the MAC 400 motor and its use with PhytO-ARM applications.

#### Getting Started

The MAC400 motor is a 400 Watt servo motor manufactured by JVL, a Danish motor/power company. The motor combines several features into a single unit: a high-torque motor to lift any of the instruments used on a deployment, an absolute encoder to track position, an Ethernet connected module that controls motor movement and provides feedback from the motor, and a durable outer case that protects the internal mechanisms from water, dust, and spray.

The motor has 2 basic parts: the motor body and the control module.

The motor body has a large heatsink on the back to help with cooling, be cautious as it will get hot! Two large M16 plugs are located on the back of the motor: one supplies power to the motor drive circuit, the other is a power dump outlet that can be used in high power applications.

The control module sits inside the opening on the top of the motor. It has a few internal pin connectors that connect it to the hardware in the motor body. Be careful to align these when inserting the module. The module is held in place with two screws on either side, and has a rubber O-ring seal around the inside to seal out moisture. 

All the connections on the control module are threaded M12 connectors. The control circuit power, LAN, and serial connectors all have slightly different hole patterns, so double check the cables before attempting to connect them to the motor. The control circuit power, Local Area Network, and serial connectors all have slightly different hole patterns, and corresponding LED lights.

The motor is easily added to a network by reserving an IP address and modifying the internal motor setting via serial cable to match the network reservation. Once this process is complete, the motor can be controlled over Ethernet using a range of Ethernet protocols including EtherNet/IP, ModbusTCP, PowerLink, EtherCAT, and others.

For the steps outlined here, you will need to use the serial connection and an RS232 to USB adapter for much of the setup. Once the motor has an IP address assigned, you can switch to using the LAN cable and connecting to the motor via Ethernet.

#### Setting up the module in Sierra Wireless ACE Manager
Note: this setup guide uses IP addresses specific to the modem and SIM card used in this deployment. Please modify the last pair of digits to match the IP address for the SIM card in use.

1. Look up MAC address for module, this will be listed on a sticker at the bottom edge

2. Log in to Sierra Wireless ACE manager and select the “LAN” tab, and expand the 3rd list titled DHCP Reservation List

![Alt](https://github.com/WHOIGit/PhytO-ARM/blob/master/website/static/images/MAC1.png)

3. Create a new entry and enter the MAC address for the module. If the motor is the only one on the network, assign it to 192.168.13.105. If there is more than 1 motor on the network, increase the IP address by 1, so 192.68.13.106, 192.168.13.107, etc. 

![Alt](https://github.com/WHOIGit/PhytO-ARM/blob/master/website/static/images/MAC2.png)

4. To set up remote access directly to the Win 10 NUC, select the “Security” tab and check the port forwarding table. There should be two entries for Win 10 NUC (IP address 192.168.13.2), one for remote desktop and one for SSH. If these aren’t there, they need to be added.

![Alt]((https://github.com/WHOIGit/PhytO-ARM/blob/master/website/static/images/MAC3.png)

  * The public start port for remote access will be 3932, the private port will be 3389 (default Remote Desktop port)
  * The public start port for SSH will be 3922, the private port will be 20 (the default SSH port)
  * The motor does not have any port forwarding set up as it should never need to be remotely accessed, all access will be through the Win 10 machine at 192.168.13.2

5. The module does not need to be connected via Ethernet for reserving the address in ACE Manager, however you do need the physical network set up for the next steps. Be sure to connect all the components via Ethernet to a switch
  * Ensure the Win10 machine, motor, and modem are all plugged into the same switch OR
  * All components are plugged in any switch that has been daisy chained to another
  * There are many components that all require power: the modem, switch, Win10 NUC, adjustable DC power supply for the motor control circuit, and AC power for the motor drive circuit, plus any additional computers or monitors.

#### MacTalk Setup & Addressing
1.	Connect to power (control circuit and motor power circuit)
2.	Connect serial cable with serial/USB adapter
  * If the motor does not appear in MacTalk, make sure you’ve selected “Serial port” from the dropdown menu in the upper left corner. Select the appropriate COM port in the next menu, and make sure the baudrate is set to the MacTalk default of 19200

![Alt](https://github.com/WHOIGit/PhytO-ARM/blob/master/website/static/images/MAC4.png)

  * If there is no green circle next to the selected COM port, open Device Manager on Windows (click the Windows icon on the bottom left and type “Device Manager”). Click “Ports (COM & LPT)” to expand the list of available ports. The USB Serial Port will have a COM port number listed next to it, select that COM port in MacTalk
  * If the green circle still doesn’t show up, restart MacTalk
3.	Change the IP settings in the Module tab
  * IP for the module should be 192.168.13.105. If there is more than 1 motor on the network, increase the IP address by 1, so 192.68.13.106, 192.168.13.107, etc.
  * Subnet is 255.255.255.0
  * Default gateway is the IP address for the modem (192.168.13.31)

![Alt]((https://github.com/WHOIGit/PhytO-ARM/blob/master/website/static/images/MAC5.png)

4.	Click Apply and Save, this will save all the changes you made, the module will reboot and the new IP address should show up in the lower right corner along with the motor icon
5.	Check the box that says Use DHCP and then Apply & Save
  * The modem will assign it the address you gave it in ACE manager using DHCP, but this gives us a backup in case the box gets unchecked or DHCP fails for some reason

#### Connecting with the IP Address
1.	Restart MacTalk. It's possible the motor will be detected automatically over Ethernet during the initial scan. 
2.	If the motor is not detected right away, open the dropdown menu on the far left. There should be a “Serial port” option and a local Ethernet connection. The Ethernet connection should be:
  * Displayed with a green icon next to it, and
  * Listed with the address 192.168.13.2, which is the address of the computer

![Alt](https://github.com/WHOIGit/PhytO-ARM/blob/master/website/static/images/MAC6.png)

3.	If you don’t see the green icon or the IP address is something else (like 10.0.0.80 or 169.254.15.44), check your Ethernet connection
  * Disconnect from any other Wi-Fi networks, and ensure your Ethernet connection is working
  * Try launching a web page. Sometimes the Ethernet connection gets reset and needs to be reconfigured. If you can’t connect but should have Internet access, run the Network Connection Troubleshooter. This should automatically reconfigure the Ethernet port and allow you access, the most common reason for this is “Ethernet doesn’t have a valid IP configuration”. Running the troubleshooter will resolve this.
  * Restart MacTalk. If you’ve just connected the computer, the network connection may still be configuring when you launch MacTalk. Close the program and reopen it, check to see if the new IP address has shown up.

#### Updating Firmware
The Ethernet module can be programmed to use a range of Ethernet protocols