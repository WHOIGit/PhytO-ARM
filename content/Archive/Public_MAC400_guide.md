---
title: "MAC 400 Motor Guide"
---

This guide serves as a reference manual for the MAC 400 motor and its use with PhytO-ARM applications.

#### Getting Started

The MAC400 motor is a 400 Watt servo motor manufactured by JVL, a Danish motor/power company. The motor combines several features into a single unit: a high-torque motor to lift any of the instruments used on a deployment, an absolute encoder to track position, an Ethernet connected module that controls motor movement and provides feedback from the motor, and a durable outer case that protects the internal mechanisms from water, dust, and spray.

The motor has 2 basic parts: the motor body and the control module.

The motor body has a large heatsink on the back to help with cooling, be cautious as it will get hot! Two large M16 plugs are located on the back of the motor: one supplies power to the motor drive circuit, the other is a power dump outlet that can be used in high power applications.

The control module sits inside the opening on the top of the motor. It has a few internal pin connectors that connect it to the hardware in the motor body. Be careful to align these when inserting the module. The module is held in place with two screws on either side, and has a rubber O-ring seal around the inside to seal out moisture. 

All the connections on the control module are threaded M12 connectors. The control circuit power, LAN, and serial connectors all have slightly different hole patterns, so double check the cables before attempting to connect them to the motor. The control circuit power, Local Area Network, and serial connectors all have slightly different hole patterns, and corresponding LED lights.

#### MacTalk Setup & Addressing

The motor is easily added to a network by reserving an IP address and modifying the internal motor setting via serial cable to match the network reservation. Once this process is complete, the motor can be controlled over Ethernet using a range of Ethernet protocols including EtherNet/IP, ModbusTCP, PowerLink, EtherCAT, and others.

For the steps outlined here, you will need to use the serial connection and an RS232 to USB adapter for much of the setup. Once the motor has an IP address assigned, you can switch to using the LAN cable and connecting to the motor via Ethernet.

1.	Connect to power (control circuit and motor power circuit)
2.	Connect serial cable with serial/USB adapter
  * If the motor does not appear in MacTalk, make sure you’ve selected “Serial port” from the dropdown menu in the upper left corner. Select the appropriate COM port in the next menu, and make sure the baudrate is set to the MacTalk default of 19200

![Alt](https://github.com/WHOIGit/PhytO-ARM/blob/master/website/static/images/MAC4.png)

  * If there is no green circle next to the selected COM port, open Device Manager on Windows (click the Windows icon on the bottom left and type “Device Manager”). Click “Ports (COM & LPT)” to expand the list of available ports. The USB Serial Port will have a COM port number listed next to it, select that COM port in MacTalk
  * If the green circle still doesn’t show up, restart MacTalk
3.	Change the IP settings in the Module tab
  * IP for the module should be 192.xxx.xx.105. If there is more than 1 motor on the network, increase the IP address by 1, so 192.xxx.xx.106, 192.xxx.xx.107, etc.
  * Subnet is 255.255.255.0
  * Default gateway is the IP address for the modem

![Alt](https://github.com/WHOIGit/PhytO-ARM/blob/master/website/static/images//MAC5.png)

4.	Click Apply and Save, this will save all the changes you made, the module will reboot and the new IP address should show up in the lower right corner along with the motor icon
5.	Check the box that says Use DHCP and then Apply & Save
  * The modem will assign it the address you gave it in ACE manager using DHCP, but this gives us a backup in case the box gets unchecked or DHCP fails for some reason

#### Connecting with the IP Address
1.	Restart MacTalk. It's possible the motor will be detected automatically over Ethernet during the initial scan. 
2.	If the motor is not detected right away, open the dropdown menu on the far left. There should be a “Serial port” option and a local Ethernet connection. The Ethernet connection should be:
  * Displayed with a green icon next to it, and
  * Listed with the computer IP address

![Alt](https://github.com/WHOIGit/PhytO-ARM/blob/master/website/static/images/MAC6.png)

3.	If you don’t see the green icon or the IP address is something that does not match your machine's IP address, check your Ethernet connection
  * Disconnect from any other Wi-Fi networks, and ensure your Ethernet connection is working
  * Try launching a web page. Sometimes the Ethernet connection gets reset and needs to be reconfigured. If you can’t connect but should have Internet access, run the Network Connection Troubleshooter. This should automatically reconfigure the Ethernet port and allow you access, the most common reason for this is “Ethernet doesn’t have a valid IP configuration”. Running the troubleshooter will resolve this.
  * Restart MacTalk. If you’ve just connected the computer, the network connection may still be configuring when you launch MacTalk. Close the program and reopen it, check to see if the new IP address has shown up.

#### Updating Firmware
The Ethernet module can be programmed to use a range of Ethernet protocols, and reprogramming the module can be done in a few minutes through MacTalk. If a seemingly functional motor is not responsive or a connection cannot be esablished over a known Ethernet connection, one common issue is having incorrect or outdated firmware on the module. 

To change or update the firmware on a module, click on the Updates menu from the top menu bar and select Update Firmware. A warning will appear that old firmware will be erased, click OK to proceed.

A pop-up window should show firmware update options. If the firmware you need is not immediately listed, check the box that says "Show all files", and select the appropriate file. Alternatively, you can browse the firmware folder using the button on the right. 

![Alt](https://github.com/WHOIGit/PhytO-ARM/blob/master/website/static/images/MAC7.png)

Choose the firmware you would like installed on the module by selecting it, and click the "Start" button at the bottom of the window. Once you start the installation, the window should display the current firmware on the module, the new version being loaded, and the download progress. The installer will run, alerting you when the installation is complete. 

![Alt](https://github.com/WHOIGit/PhytO-ARM/blob/master/website/static/images/MAC8.png)

Once the firmware is updated, click the "Exit" button to close the window. MacTalk will restart automatically, and the module tab will now display the new firmware.



#### Tuning the Motor

The motor is configured to behave differently depending on the other hardware it has been paired with. In our setups, we use a planetary gearbox, which is highly precise and most commonly used with servo motors. Given the high level of precision in the hardware, we need to account for that in the MacTalk setup. 

If left on the default settings, the motor will quickly begin skipping or making lots of shaky, jerky movements. This is because the default tolerance for follow error, or the number of counts between the target and actual count, is nearly 0. Servo motors will generally make several small, rapid adjustments to maintain a position or speed, which can be useful in many other applications; here we want a smooth ascent and descent with precise stopping points. 

To fix this, we need to make 2 adjustments. First, adjust the motor tuning by selecting the "Filter Setup" icon in MacTalk. This will bring up a small floating menu.

![Alt](https://github.com/WHOIGit/PhytO-ARM/blob/master/website/static/images/MAC9.png)

Here, we need to adjust the Position/Velocity filter. The default setting will be at the 5,3 position on the x,y grid. This needs to move to the 3,3 position, where there is more stability but still relatively quick corrections to the movement. After this has been changed, click "Load Filter" to save the changes. Check to make sure the new filter is in place by closing the window and reopening it to ensure the same setting are selected.

Secondly, we need to open up the In Position Window for the motor. With more stability and a quick response time, the motor occasionally will approach the target count and get close, but not within the 100 counts needed to be "In Position". When this happens, the motor does not reach the target position and any script needing the "In Position" bit feedback will be left running without ending.

To change the In Position Window, we need to put the motor into Position Mode. This can be done by either changing the actual mode, or you can uncheck the box next to "Change Actual Mode".

![Alt](https://github.com/WHOIGit/PhytO-ARM/blob/master/website/static/images/MAC10.png)

Once in Position Mode, a box will appear under Motion Parameters that says "In position window". The default value is 100, enter 200 in the box and hit enter. This will update the register for this value. If you want to be sure your changes have been saved, click "Save in Motor" along the top menu bar. This will save the register values to the motor and restart the module. Confirm when you switch into Position Mode that this value remains set at 200.

#### Running the Motor in MacTalk
- Select the mode in the upper left menu. If you want to look at the settings in a different mode but not have the motor move, uncheck the box that says “Change Actual Mode”. After adjusting the settings for that mode, make sure you are in the original mode and check the box to enable mode switching again
- Flipping between position & passive, the encoder jumps 10-15 counts, this is normal and is a result of the motor no longer using power to maintain position. This change is also extremely small in terms of distance from the target position (8192 counts per rotation)
- Use the toggle arrows next to the values to adjust target speed, position, acceleration, and other values in real time

##### Motor Status
The Status bar at the top of the will show most of the information about the motor status.
Some key values:

- Actual Mode: the mode the motor is currently set to (Passive, Position, Velocity, etc.)
- Actual Position: the current position of the encoder
- Motor Load (mean): motor load is the I2T value that is reported in register 16, and cannot exceed 100%. Mean load is related to torque; if the motor load is too great and torque reaches 300%, the motor will enter passive mode
- Temperature: motor internal temperature. While testing, the temperature levels out around 38/40°C. If the temperature exceeds 84°C/183°F, there will be a Temperature Too High error. See Errors & Troubleshooting for more details

##### Inputs
- Bus Voltage: Voltage in the motor circuit. This value is in VDC and will be higher than your VAC input of 115VAC. A typical range would be about 350-380VDC, an Overvoltage on bus error will occur if the internal bus voltage exceeds 450VDC. See Errors & Troubleshooting for more details
- Control Voltage: Voltage in the control circuit

##### Errors
All errors will be displayed in the Errors bar above Status. You may need to expand the list by clicking the red arrow to display any errors.

##### Status
This section displays a list of status readings and warnings, current statuses will appear in bold. If any errors occur they will appear in the Errors panel directly above.

- Motor in position: the motor is at the specified position (in Position mode)
- Control Voltage: Voltage in the control circuit
- Motor is accelerating/decelerating: either of these can be bolded when the motor is functioning depending on the direction of motion (accelerating is clockwise, decelerating is counter clockwise)
- Intermediate Power DC voltage low: this will appear when only the control circuit has power (or power supplied to the motor is too low). This should go away when AC voltage is applied to the motor circuit. If it does not, check the control circuit voltage reading, as it may be below the required range, which will strain the motor and ultimately cause problems leading to a shutdown
- Supervision of position limits disabled: position limits will be temporarily disabled if the motor runs outside the position limits set in the registers. Supervision is re-enabled when the motor comes back inside the position limits that were set
- VAC On: AC voltage is supplied to the motor. This status message will be on unless AC power is removed, and is one of the constant messages displayed during normal operations
- Driver stage disabled: An error has occurred and the motor will not run properly

#### Errors & Troubleshooting

Common Errors

##### Connection to the motor over Python script fails

|Possible Cause            | Solution |
| ---- | ---- |
|Motor is not powered appropriately      |Check that the control circuit is connected to adequate power (18-30VDC), and the motor circuit is plugged in to AC power. Alternatively check that battery levels are sufficient      |
|Incorrect or out of date firmware is installed on the module      |Make sure the module has the correct firmware installed      |
| Computer can't connect to the right port                     |The default port is 502, be sure this is the requested port. Some computers have trouble recognizing serial ports--check your device manager to ensure one is set up      |
| IP address is incorrect                          |Try to ping the motor from the command line. If it doesn't respond, check the IP address in MacTalk. <br />Reset your network connection or reconfigure the IP address using the serial connection |

##### Motion is jerky or seems out of control

| Possible Cause                                               | Solution                                                     |
| ------------------------------------------------------------ | ------------------------------------------------------------ |
| Position controls are on                                     | If the motor is position-limited, it will stop if directed to exceed the limit. Return the motor to the limits and/or disable the limits |
| The tuning has not been adjusted for the current load        | Adjust the load factor in MacTalk or by changing the register value |
| Some other setting has been inappropriately changed (this has happened before) | Do a complete reset of the motor in MacTalk                  |

##### I can't identify a problem but the motor does not run

| Possible Cause                                               | Solution                                                     |
| ------------------------------------------------------------ | ------------------------------------------------------------ |
| Insufficient power                                           | If battery levels are low, the motor may have enough power to stay on, but not enough to run. Switch into position mode and see if a power error pops up in the lower right corner. Let the system charge and try again |
| Previous cast did not end normally or there was an error during the cast | If using the script, an error on the previous cast may cause issues. If the script is still running, the next profile won't be able to connect to the motor or RBR. Stop the running task, disable the RBR, and try again |
| Incorrect firmware installed on the module                   | Make sure the module has the correct firmware installed      |

