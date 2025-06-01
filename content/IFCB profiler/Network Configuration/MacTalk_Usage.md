---
title: "MacTalk Usage"
weight: 7
---

#### Updating Firmware

The Ethernet module can be programmed to use a range of Ethernet protocols, and reprogramming the module can be done in a few minutes through MacTalk. If a seemingly functional motor is not responsive or a connection cannot be established over a known Ethernet connection, one common issue is having incorrect or outdated firmware on the module.

To change or update the firmware on a module, click on the Updates menu from the top menu bar and select Update Firmware. A warning will appear that old firmware will be erased, click OK to proceed.

A pop-up window should show firmware update options. If the firmware you need is not immediately listed, check the box that says "Show all files", and select the appropriate file. Alternatively, you can browse the firmware folder using the button on the right.

![Alt](/images/MAC7.png)

Choose the firmware you would like installed on the module by selecting it, and click the "Start" button at the bottom of the window. Once you start the installation, the window should display the current firmware on the module, the new version being loaded, and the download progress. The installer will run, alerting you when the installation is complete. NOTE: Do not stop this process midway through, it is possible to brick a unit by interupting this installation process. 

![Alt](/images/MAC8.png)

Once the firmware is updated, click the "Exit" button to close the window. MacTalk will restart automatically, and the module tab will now display the new firmware.

#### Tuning the Motor

The motor is configured to behave differently depending on the other hardware it has been paired with. In our setups, we use a conedrive gearbox, which is highly precise and most commonly used with servo motors. Given the high level of precision in the hardware, we need to account for that in the MacTalk setup.

If left on the default settings, the motor will quickly begin skipping or making lots of shaky, jerky movements. This is because the default tolerance for follow error, or the number of counts between the target and actual count, is nearly 0. Servo motors will generally make several small, rapid adjustments to maintain a position or speed, which can be useful in many other applications; here we want a smooth ascent and descent with precise stopping points.

To fix this, we need to make 2 adjustments. First, adjust the motor tuning by selecting the "Filter Setup" icon in MacTalk. This will bring up a small floating menu.

![Alt](/images/MAC9.png)

Here, we need to adjust the Position/Velocity filter. The default setting will be at the 5,3 position on the x,y grid. This needs to move to the 3,3 position, where there is more stability but still relatively quick corrections to the movement. After this has been changed, click "Load Filter" to save the changes. Check to make sure the new filter is in place by closing the window and reopening it to ensure the same setting are selected.

Secondly, we need to open up the In Position Window for the motor. With more stability and a quick response time, the motor occasionally will approach the target count and get close, but not within the 100 counts needed to be "In Position". When this happens, the motor does not reach the target position and any script needing the "In Position" bit feedback will be left running without ending.

To change the In Position Window, we need to put the motor into Position Mode. This can be done by either changing the actual mode, or you can uncheck the box next to "Change Actual Mode".

![Alt](/images/MAC10.png)

Once in Position Mode, a box will appear under Motion Parameters that says "In position window". The default value is 100, enter 200 in the box and hit enter. This will update the register for this value. If you want to be sure your changes have been saved, click "Save in Motor" along the top menu bar. This will save the register values to the motor and restart the module. Confirm when you switch into Position Mode that this value remains set at 200.

#### Running the Motor in MacTalk

-   Select the mode in the upper left menu. If you want to look at the settings in a different mode but not have the motor move, uncheck the box that says "Change Actual Mode". After adjusting the settings for that mode, make sure you are in the original mode and check the box to enable mode switching again
-   Flipping between position & passive, the encoder jumps 10-15 counts, this is normal and is a result of the motor no longer using power to maintain position. This change is also extremely small in terms of distance from the target position (8192 counts per rotation)
-   Use the toggle arrows next to the values to adjust target speed, position, acceleration, and other values in real time

##### Motor Status

The Status bar at the top of the will show most of the information about the motor status. Some key values:

-   Actual Mode: the mode the motor is currently set to (Passive, Position, Velocity, etc.)
-   Actual Position: the current position of the encoder
-   Motor Load (mean): motor load is the I2T value that is reported in register 16, and cannot exceed 100%. Mean load is related to torque; if the motor load is too great and torque reaches 300%, the motor will enter passive mode
-   Temperature: motor internal temperature. While testing, the temperature levels out around 38/40°C. If the temperature exceeds 84°C/183°F, there will be a Temperature Too High error. See Errors & Troubleshooting for more details

##### Inputs

-   Bus Voltage: Voltage in the motor circuit. This value is in VDC and will be higher than your VAC input of 115VAC. A typical range would be about 350-380VDC, an Overvoltage on bus error will occur if the internal bus voltage exceeds 450VDC. See Errors & Troubleshooting for more details
-   Control Voltage: Voltage in the control circuit
