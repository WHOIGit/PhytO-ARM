---
title: "MacTalk Setup"
weight: 6
---

#### MacTalk Setup & Addressing

Note: The JVL module does not need to be connected via Ethernet for reserving the address in ACE Manager, however you do need the physical network set up for the next steps. Be sure to connect all the components via Ethernet to a switch

-   Ensure the Win10 machine, motor, and modem are all plugged into the same switch OR
-   All components are plugged in any switch that has been daisy chained to another
-   There are many components that all require power: the modem, switch, Win10 NUC, adjustable DC power supply for the motor control circuit, and AC power for the motor drive circuit, plus any additional computers or monitors.

1.  Connect to power (control circuit and motor power circuit)
2.  Connect serial cable with serial/USB adapter to the JVL coms board and the win10 machine
3.  Open MacTalk on the windows machine

-   If the motor does not appear in MacTalk, make sure you've selected "Serial port" from the dropdown menu in the upper left corner. Select the appropriate COM port in the next menu, and make sure the baudrate is set to the MacTalk default of 19200

![Alt](/images/MAC4.png)

-   If there is no green circle next to the selected COM port, open Device Manager on Windows (click the Windows icon on the bottom left and type "Device Manager"). Click "Ports (COM & LPT)" to expand the list of available ports. The USB Serial Port will have a COM port number listed next to it, select that COM port in MacTalk
-   If the green circle still doesn't show up, restart MacTalk

3.  Change the IP settings in the Module tab

-   IP for the module should be 192.168.13.3.
-   Subnet is 255.255.255.0
-   Default gateway is the IP address for the modem (192.168.13.31)

![Alt](/images/MAC5.png)

4.  Click Apply and Save, this will save all the changes you made, the module will reboot and the new IP address should show up in the lower right corner along with the motor icon
5.  Check the box that says Use DHCP and then Apply & Save

-   The modem will assign it the address you gave it in ACE manager using DHCP, but this gives us a backup in case the box gets unchecked or DHCP fails for some reason

#### Connecting with the IP Address

1.  Restart MacTalk. It's possible the motor will be detected automatically over Ethernet during the initial scan.
2.  If the motor is not detected right away, open the dropdown menu on the far left. There should be a "Serial port" option and a local Ethernet connection. The Ethernet connection should be:

-   Displayed with a green icon next to it
-   Listed with the address 192.168.13.2, which is the address of the computer
-   You may need to manual enter the IP address you assigned the motor you are trying to connect into the box to the right of the wrench and driver icon

![Alt](/images/MAC6.png)

3.  If you don't see the green icon or the IP address is something else (like 10.0.0.80 or 169.254.15.44), check your Ethernet connection

-   Disconnect from any other Wi-Fi networks, and ensure your Ethernet connection is working
-   Try launching a web page. Sometimes the Ethernet connection gets reset and needs to be reconfigured. If you can't connect but should have Internet access, run the Network Connection Troubleshooter. This should automatically reconfigure the Ethernet port and allow you access, the most common reason for this is "Ethernet doesn't have a valid IP configuration". Running the troubleshooter will resolve this.
-   Restart MacTalk. If you've just connected the computer, the network connection may still be configuring when you launch MacTalk. Close the program and reopen it, check to see if the new IP address has shown up.
