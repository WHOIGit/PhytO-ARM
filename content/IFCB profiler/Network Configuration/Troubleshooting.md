---
title: "Troubleshooting"
Weight: 9
---

##### Errors

All MacTalk errors will be displayed in the Errors bar above Status in the MacTalk program. You may need to expand the list by clicking the red arrow to display any errors.

##### Status

This section displays a list of status readings and warnings, current statuses will appear in bold. If any errors occur they will appear in the Errors panel directly above.

-   Motor in position: the motor is at the specified position (in Position mode)
-   Control Voltage: Voltage in the control circuit
-   Motor is accelerating/decelerating: either of these can be bolded when the motor is functioning depending on the direction of motion (accelerating is clockwise, decelerating is counter clockwise)
-   Intermediate Power DC voltage low: this will appear when only the control circuit has power (or power supplied to the motor is too low). This should go away when AC voltage is applied to the motor circuit. If it does not, check the control circuit voltage reading, as it may be below the required range, which will strain the motor and ultimately cause problems leading to a shutdown
-   Supervision of position limits disabled: position limits will be temporarily disabled if the motor runs outside the position limits set in the registers. Supervision is re-enabled when the motor comes back inside the position limits that were set
-   VAC On: AC voltage is supplied to the motor. This status message will be on unless AC power is removed, and is one of the constant messages displayed during normal operations
-   Driver stage disabled: An error has occurred and the motor will not run properly

#### Errors & Troubleshooting

Common Errors

##### Connection to the motor over Python script fails

| Possible Cause                                               | Solution                                                                                                                                                                                           |
|--------------------------------------------------------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| Motor is not powered appropriately                           | Check that the control circuit is connected to adequate power (18-30VDC), and the motor circuit is plugged in to AC power. Alternatively check that battery levels are sufficient                  |
| Incorrect or out of date firmware is installed on the module | Make sure the module has the correct firmware installed                                                                                                                                            |
| Computer can't connect to the right port                     | The default port is 502, be sure this is the requested port. Some computers have trouble recognizing serial ports--check your device manager to ensure one is set up                               |
| IP address is incorrect                                      | Try to ping the motor from the command line. If it doesn't respond, check the IP address in MacTalk. <br />Reset your network connection or reconfigure the IP address using the serial connection |

##### Motion is jerky or seems out of control

| Possible Cause                                                                 | Solution                                                                                                                                 |
|--------------------------------------------------------------------------------|------------------------------------------------------------------------------------------------------------------------------------------|
| Position controls are on                                                       | If the motor is position-limited, it will stop if directed to exceed the limit. Return the motor to the limits and/or disable the limits |
| The tuning has not been adjusted for the current load                          | Adjust the load factor in MacTalk or by changing the register value                                                                      |
| Some other setting has been inappropriately changed (this has happened before) | Do a complete reset of the motor in MacTalk                                                                                              |
erm
##### I can't identify a problem but the motor does not run

| Possible Cause                                                           | Solution                                                                                                                                                                                                                  |
|--------------------------------------------------------------------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| Insufficient power                                                       | If battery levels are low, the motor may have enough power to stay on, but not enough to run. Switch into position mode and see if a power error pops up in the lower right corner. Let the system charge and try again   |
| Previous cast did not end normally or there was an error during the cast | If using the script, an error on the previous cast may cause issues. If the script is still running, the next profile won't be able to connect to the motor or RBR. Stop the running task, disable the RBR, and try again |
| Incorrect firmware installed on the module                               | Make sure the module has the correct firmware installed                                                                                                                                                                   |
