# NUC Information & Setup
The on-board computer is the fundamental control device for the winch systems. The NUC is a compact PC running Windows10 and acts as the control device for all the onboard winches. There are few programs required to control the winches:
-	TeraTerm: Serial port software for communicating with the RBR via serial cable
-	Serial Port Monitor: monitoring software to capture traffic between the computer and RBR via serial cable
-	Ruskin: RBR software that allows analysis and display of binary data, can also be used for real-time data streaming
-	MacTalk: MAC motor-specific software for setup and control of the MAC400 motors
-	Python version 2.14.7: initially installed to be compatible with previous code iterations, we chose to stick with Python 2 for ROS compatibility. At time of writing, there are no known conflicts that would prevent running on Python 3.
-	Git: critical version-control software for all changes to Python code

There are a few critical modifications to the system settings to allow the NUC to behave properly in the field. These are enabling Remote Desktop connections and allowing the machine to reboot on power cycle.

#### Remote Desktop

To allow Remote Desktop connections at any time, modify the System settings:

-	Modify the sleep & power settings to prevent the machine from sleeping
  -	For screen settings, allow 10 minutes of inactivity before the screen is turned off
  -	Under "Sleep", when the PC is plugged in, selected "Never" for time to sleep
-	Under remote desktop settings, click the toggle switch to allow Remote Desktop connections.
  -	Enable "keep my PC awake for connections when it is plugged in" 
  -	Enable "make my PC discoverable on private networks to enable automatic connection from a remote device"

#### Reboot on Power Cycle

The machine will need to be able to reboot on power cycle, which requires modifying the BIOS on the computer. Full instructions are [here](https://www.laptopmag.com/articles/access-bios-windows-10).

To do this, go to Settings > Updates & Security > Recovery and click "Restart now" under Advanced startup. 

From the options on the next screen, select "Troubleshoot", then "Advanced Options". Finally, select "UEFI Firmware settings" and click "Restart". This will take you to the BIOS on restart.

![Alt](https://github.com/WHOIGit/PhytO-ARM/blob/master/website/static/images/NUC1.png)

Once in BIOS, click the "Advanced" tab and then select the "Power" tab towards the right of the screen. Under Secondary Power settings, locate "After Power Failure". From the pull down menu, select Power On. 

![Alt](https://github.com/WHOIGit/PhytO-ARM/blob/master/website/static/images/NUC2.png)

From there, you can hit F10 to save and exit, or click the 'x' in the upper right corner. A popup will ask if you want to save, click yes and continue.

After the computer restarts, it should boot up every time AC power is removed and put back, allowing the computer to be power cycled remotely.

#### Automatic Wi-Fi Connection

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
