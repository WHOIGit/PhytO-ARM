---
title: "Software Setup & Reference Guide"
---

The on-board computer is the fundamental control device for the winch systems. The NUC is a compact PC running Windows10 and acts as the control device for all the onboard winches. There are few programs required to control the winches:
-	TeraTerm: Serial port software for communicating with the RBR via serial cable
-	Serial Port Monitor: monitoring software to capture traffic between the computer and RBR via serial cable
-	Ruskin: RBR software that allows analysis and display of binary data, can also be used for real-time data streaming
-	MacTalk: MAC motor-specific software for setup and control of the MAC400 motors. See the Mac Motor Guide for more information.
-	Python version 2.14.7: initially installed to be compatible with previous code iterations, we chose to stick with Python 2 for ROS compatibility. At time of writing, there are no known conflicts that would prevent running on Python 3.
-	Git: critical version-control software for all changes to Python code

### Software & Settings
There are a few critical modifications to the system settings to allow the NUC to behave properly in the field. These include allowing for remote access, switching the device on and off, and managing energy saving settings that interfere with background tasks.

#### Remote Desktop

To allow Remote Desktop connections at any time, modify the System settings:

-	Modify the sleep & power settings to prevent the machine from sleeping
  -	For screen settings, allow 10 minutes of inactivity before the screen is turned off
  -	Under "Sleep", when the PC is plugged in, selected "Never" for time to sleep
-	Under remote desktop settings, click the toggle switch to allow Remote Desktop connections.
  -	Enable "keep my PC awake for connections when it is plugged in" 
  -	Enable "make my PC discoverable on private networks to enable automatic connection from a remote device"

#### Reboot on Power Cycle

Occasionally, we need to be able to shut down all on-board systems when power is low. The machine will need to be able to reboot on power cycle, which requires modifying the BIOS on the computer. Full instructions are [here](https://www.laptopmag.com/articles/access-bios-windows-10).

To do this, go to Settings > Updates & Security > Recovery and click "Restart now" under Advanced startup. 

From the options on the next screen, select "Troubleshoot", then "Advanced Options". Finally, select "UEFI Firmware settings" and click "Restart". This will take you to the BIOS on restart.

![Alt](/images/NUC1.jpeg)

Once in BIOS, click the "Advanced" tab and then select the "Power" tab towards the right of the screen. Under Secondary Power settings, locate "After Power Failure". From the pull down menu, select Power On. 

![Alt](/images/NUC2.jpeg)

From there, you can hit F10 to save and exit, or click the 'x' in the upper right corner. A popup will ask if you want to save, click yes and continue.

After the computer restarts, it should boot up every time AC power is removed and put back, allowing the computer to be power cycled remotely.

#### Power Management

One of the standard Windows power-saving features disconnects the computer from the network when the computer is in sleep mode. While perfectly acceptable in an office setting where you can easily wake the computer, this will prevent the winch profile from running correctly if this feature is not disabled.

There are several ways to prevent the computer from losing the network connection when it sleeps. The easiest way is to disable the sleep feature altogether. If the computer can never go to sleep, it will never disconnect from the network.

![Alt](/images/NUC3.png)

Since the computer is always plugged in (the NUC has no battery backup), you can change the Sleep setting to 'Never' instead of 10 or 20 minutes. However, it is best to disable the feature that would cause the disconnect in the first place. 

The best way to do this varies slightly based on what version of Windows your computer is running. In some instances, you can do this easily though "Settings" > "Display" > "Power and Sleep". For most computers running Windows 10, this is no longer an option, so you need to modify the power settings another way.

From the Start Menu, type "Edit Group Policy", which should pull up a Control Panel window with the Local Group Policies. 

Once in the Local Group Policy Editor, navigate to Sleep Settings:
Computer Configuration -> Administrative Templates -> System -> Power Management -> Sleep Settings

The specific setting is "Allow network connectivity during connected-standby", and should show options for both battery power and plugged in. Double click on the setting, and check the "Enable" box in the upper left of the window that pops up. Click "Apply" and "Ok" to save your changes.


#### Automatic Wi-Fi Connection (Optional)

If using the RBR on-board Wi-Fi, there may be issues connecting to the network automatically, as Windows 10 has issues connecting to open networks, regardless of the setting you choose for the network. The RBR Wi-Fi networks are all open, and the computer will not automatically connect even if you select "Connect automatically" when it pops up. There is a fix, but it is non-obvious. Details can be found [here](https://appuals.com/windows-10-will-not-connect-to-wifi-automatically/)

Here are the steps to try:

1. Try forgetting the network and reconnecting with the "Connect automatically" box checked
  * In Settings, under Wireless Network Connections, select Manage Known Networks, find the network and click "Forget"
  * Restart the computer and reconnect, being sure to check the "Connect automatically" box. You can confirm this under network properties once connected
2. Sometimes the computer will turn off the network adapter to save power
  * To disable this, click the start menu and type "Device Manager". Click on the Device Manager icon and select "Network Adpaters" from the list. Right click and select "Properties" from the menu.
  * Under the "Power Management" tab, uncheck the box next to "Allow the computer to turn off this device to save power" and click "Okay" to save
  * Restart the computer and check if the network will connect automatically
3. Modify the Group Policy in the Registry Editor, or create one if there isn't one
  * In the start menu, type "regedit" and select the Registry Editor App
  * Navigate to HKEY_LOCAL_MACHINE\Software\Policies\Microsoft\Windows\WcmSvc, and check for a GroupPolicy. If one doesn't exist, right click on WcmSvc and choose "New", select "Key" and name it GroupPolicy. 
  * Click on GroupPolicy, and right click on the right pane. Choose "New", select DWORD (32 bit), and create the value. Name it fMinimizeConnections, click "Okay" to save and then reboot the computer.

### Setting Up the Profiler

Setting up the profiler on a new computer is fairly straightforward. The code needs to be downloaded from the GitHub repository, Tera Term needs to be configured to test the RBR, and a task to run the profile needs to be created.

#### Cloning a GitHub Repository

GitHub makes sharing code and documentation extremely easy. You can either download a zip file of the repository or use the command line. 

To clone the repository using the command line, use the 'git clone' command. A full step-by-step can be found in the GitHub Docs, [here](https://docs.github.com/en/github/creating-cloning-and-archiving-repositories/cloning-a-repository-from-github/cloning-a-repository). Once the repository is copied, make sure all relevant packages and dependencies have been installed. In the future we aim to have a requirements file that will automatically download the appropriate versions and dependencies.

#### Configuring Tera Term

While you can choose to configure the Tera Term window each time you use it to connect to the RBR, it will save time and frustration if there is a configuration file you can load in with all your settings.

Launch Tera Term and select the appropriate COM port under the Serial option. There may be more than one COM port available, so be sure to select the correct one.

![Alt](/images/NUC4.png)

Once connected, select the Setup menu at the top of the terminal. You need to configure both the Terminal as well at the Serial Port settings.

Under Setup, click on Terminal Setup. Most of the default settings are matched to the default settings on the RBR, but we want to be able to view the commands we send. To do this, check the box next to 'Local Echo' and click 'OK'. This will echo each command sent so you can see both the command and the response from the logger.

![Alt](/images/NUC5.png)

Next, you need to adjust the baud rate, as the RBR uses a much higher baud rate than the Tera Term default. From the Setup menu, select Serial Port Setup and Connection (typically just displayed as Serial Port...). Double check that the COM port is correct, and click on the drop down menu next to Speed. Switch the baud rate from 9600 to 115200, then click 'New setting'.

![Alt](/images/NUC6.png)

Now that the terminal echos commands and the baud rate is correct, you can test this by sending a few simple commands:

```
>A
>Ready:
>id
>Ready: id
```

To save this configuration, select "Save setup" under the Setup tab, naming it RBR. 

![Alt](/images/NUC7.png)

When using Tera Term to communicate with the RBR, you can load this saved configuration instead of manually adjusting the settings each time. Once you open the terminal, go the the Setup tab and click "Restore setup". The saved 

#### Setting Up an Automatic Task

The profiler runs every 20 minutes (or other programmable interval) thought Windows Task Scheduler. Task Scheduler is a bit clunky to use, but has many useful features. 

To set up the task, open Task Scheduler (type "Task Scheduler" in the Start menu) and select "Create Task" from the sidebar on the right. The ''General" tab will open by default.

Fill in the relevant information for the task: give it a name and a brief description, then make sure it has the right permissions. Select "Run whether user is logged on or not", and check the box next to "Run with highest privileges". Be sure to check the user is an admin to prevent any permissions problems down the road!

![Alt](/images/NUC8.png)

Under the Trigger tab, select "New Trigger" to set up the parameters that will trigger the script to run. There are a lot of ways to set the task to run every X minutes, but this is what we've found to be the most straightforward.

The option to begin the task "On a schedule" should be selected by default. Set the task to start Daily, enter the current date and midnight /12:00:00 AM as the time. It should be set by default to recur every day, but make sure that value is set to "1".

Under the advanced settings, check the box next to "Repeat task every:" and use the drop down menu to select the appropriate interval. Our deployments sample every 20 minutes, which is not an option on this menu; to enter a custom time, type it directly in the box, e.g. "20 minutes", "1320 seconds", "2 hours", etc.

Before closing out of the trigger configuration, make sure there is no delay on the task, and there is no expiration date for the task. Lastly, confirm that the box next to "Enabled" is checked. The task will not execute if this is left blank!

![Alt](/images/NUC9.png)

After completing the trigger, select the Action Tab, and create a new action. The default action is "Start a program", so we only need to specify which program to run. 

Since we're running a Python script, we need to specify what program to open the script in, which is python.exe. Under "Program/script", fill in the path to python.exe on your computer. To direct it to the correct script, enter the name of the script in the "Add arguments" box. The script is located in the cloned repository rather than with the rest of the Python library, so we direct the task to that as well by inputting the path to the script in the "Start in" box.

![Alt](/images/NUC10.png)

Once all this is in place, save your changes. Since it runs with highest privileges, you need to save the changes with an admin account. On some machines, you need to differentiate between the local account and the admin account. To do this, type  "Computer Name\User Name" in the account box, e.g. URANIA\Hablab or BUCCINUM\Hablab, then enter the password to save the changes.

### Configuring the Scripts 

Modifying the config files

Changing the target depth & speed

Note on direction: know your hardware!

### Troubleshooting 

Between power, hardware, and software, there are a number of reasons the profiler may have issues. Before deploying a new system, try to test the system as many times as possible to work out any bugs.

#### Profiler Won't Start

If the profile won't start, there are several potential causes. The issue may be a software setting or configuration, bad network connection, or a hardware disconnect.

| Issue                                                        | Possible Fixes                                               |
| ------------------------------------------------------------ | ------------------------------------------------------------ |
| Profile does not begin at scheduled time                     | Confirm the task is enabled<br />Check that the task is set to repeat on the correct schedule<br />Check to make sure all devices are on<br />Try running the script outside Task Scheduler: check for errors<br />If this happens after an email/text alert, power may be too low to switch the motor into position mode. Check power levels and try when there's a better state of charge |
| Script returns 'nonetype object is not iterable' error or 'Cannot connect to 192.168.xx.xx' | Both indicate the NUC cannot locate the motor module (these errors may co-occur). The 'nonetype' error occurs because the script queries the motor registers. If the motor fails to connect, the query returns an N/A<br />Check that the motor module is on and has adequate power.<br />Open MacTalk and check the firmware on the module. Most of the time this error occurs because the wrong firmware has been loaded.<br />Update the module firmware |
| Script stalls out when connecting to the RBR                 | Check that the RBR is properly connected and has power<br />The serial port can only accept one connection at a time. Make sure any Tera Term or Ruskin windows have been closed |
|                                                              |                                                              |
|                                                              |                                                              |



#### File Issues

| Issue                                                        | Possible Fixes                                               |
| ------------------------------------------------------------ | ------------------------------------------------------------ |
| Binary files won't open in Ruskin                            | Try importing after opening Ruskin, selecting the file and choosing "Open with Ruskin" will not work<br />The file may be formatted incorrectly. Check that the settings in Ruskin are for Desktop and not Mobile |
| The profiler was working, but my most recent files only have headers and no data |                                                              |
|                                                              |                                                              |

#### Motor Errors

| Issue                     | Possible Fixes |
| ------------------------- | -------------- |
| Motor won't connect       |                |
| Motor reports fatal error |                |
|                           |                |



#### How to Deal with an Error Text/Email



Steps when you get an alert: 
End/disable task
Open Tera Term and disable the RBR
Open MacTalk/IP cam to see where it is
Fix error/pull it up if possible

If power too low and can't restart, shut down

Profiler won't start:
Actually enabled? 
Module firmware is correct?
Devices are on?
Script is where you said?
Adequate power?



Profiler is stuck running:
Close Ruskin
Task is not actually running, it just says that. Close task scheduler
If you got an error notification, it will try to run