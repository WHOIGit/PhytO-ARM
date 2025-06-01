# PhytO-ARM: *Alma* Guide

#### Purpose of the Guide

This guide was created alongside the updates to the raft platform, now known as *Alma*, to enable new users to understand and interact with the platform, instruments, and remote interface. Much of the information covered in this guide builds on other information, skills, and equipment commonly used in the Brosnahan lab. Please refer to documentation for the individual instruments, network components, and additional hardware for more information on their operation.

By no means is this an exhaustive guide, and it should be updated frequently and thoroughly to include any changes made over time to the system. 

#### Sampling Goals

*Alma* has been deployed annually in Salt Pond in Nauset since 2012, supporting IFCB, ESP, and CTD profiling. At various points, three winches have been used in surveys, though the most recent iteration has only two installed at the time of writing. See the guide on assembly for full details on winch installation.

#### Power
The core power source on *Alma* is the bank of three 12VDC batteries wired in parallel. These are deep-cycle marine batteries that can be discharged more than other 12V batteries like car batteries. The battery bank was replaced before the 2020 field season, and should last several years before new batteries are needed.

The batteries are charged off the Blue Sky Energy solar charge controller, their status can be monitored from the Blue Sky Webpage: IP address: 166.130.161.5:235.

#### Hardware 

##### Intel Windows10 NUC 
The on-board computer is the fundamental control device for the winch systems. The NUC is a compact PC running Windows10 and acts as the control device for all the onboard winches. See the NUC setup guide for more information about the programs and settings needed for deployment.

##### MAC400 Motor
The onboard motor is a high-precision servo motor that controls profile movements and provides motion feedback over Ethernet. More details on the motor, setup, and use can be found in the MAC motor guide.

##### Sierra Wireless Modem/Router
The modem/router device allows for network connections and for proper traffic direction. The SIM card allows the modem to access the cell network to gain internet access, which in turn means the fixed IP address of the SIM card can be used to remotely access network devices from beyond the network. All devices on the Local Area Network (LAN) can be accessed through public ports using the Wide Area Network (WAN). Entering the public IP address of the modem along with the specified port allows the modem to redirect the request to the appropriate device, as shown in the example diagram below.

![Alt](https://github.com/WHOIGit/PhytO-ARM/blob/master/website/static/images/Alma1.png)

The Sierra Wireless modem is the source of your connection to the entire system and makes remote monitoring possible. For further discussion of IP reservations, port forwarding, and other features, refer to the user manual on Observatory, found under Manuals -> sierrawireless -> MP70.

The WAN address for all modems is noted in the SIM card/modem tracking sheet on Observatory, and is also typically written on a tape label on the modem itself. The password for all the Wi-Fi networks and the login for ACEmanager is the default WHOI password. You can access ACEmanager through either the LAN if working on the platform or remotely through the WAN using the LAN or WAN IP address and port 9443, *i.e* 192.168.13.31:9443 and 166.130.161.5:9443.

##### Netgear Switches
Switches are largely self-explanatory, plug-and-play, and are the major point of network connections. The Web-I Ethernet cable should be plugged in to the modem directly, but most other connections are through switches. If you run into issues with network connections, check the switches to ensure they are on and that the lights around the device cable are illuminated. If they are off or only one comes on, that’s an indication that the device is off or operating in a restricted capacity.

##### Switchable Relays
There are three onboard relay boards that control the instruments and devices on the raft. Each board has a current limit (30 amps, 10 amps, 5 amps) that controls how much current each device can draw and the boards are all powered off the 12V DC/DC converter. 

The 5 amp board supports eight switches, while the 10 and 30 amp boards both support four switches. There are three terminal block pins on each switch: NC, COM, and ON. If you ever get confused about which is which, there is a small key in the corner of the board indicating which terminal is which. 

![Alt](https://github.com/WHOIGit/PhytO-ARM/blob/master/website/static/images/Alma2.jpeg)

The cable running to the device connects to the ON terminal so that when the relay is on, the connection between power and the device is open. The COM terminal is where power from the DC/DC converter is connected, supplying power to the switch that is only connected when the relay is switched on. While each of the boards run on 12V, any device run off a relay needs a power supply run through the relay connection. The exact voltage required for each device will vary, and it is important to use the correct voltage for each device regardless of what other voltages are used on the board! For example, the winches and RBR power are all 28V, while the IP cameras are all 12V. Despite the difference in required voltages, these devices are all run off the 5 amp board.

#### Spring Assembly 

##### Power System Setup
The Blue Sky Energy controller consists of a master and two slave charging units that are linked to a charge controller by an RJ12 telephone connector. The charging units plug directly into the solar panels mounted on the housing roof. The charge controller monitors charge state, input and output, and publishes this information to a webpage.

If they are not already in place, the batteries need to be seated underneath the interior bench. Remove the seat and set the batteries in so they are held in place by the 2x4 blocking. Connect the batteries in parallel, connecting the charge controller to the positive terminal on one of the batteries. 

The charge controlelr

##### Controlling Devices through the Relays
The relays are controlled through a Web-I dongle on the 30 amp board that is connected to the network. This allows remote access to all the devices, booting up, shutting down, or power cycling as needed during deployment.

![Alt](https://github.com/WHOIGit/PhytO-ARM/blob/master/website/static/images/Alma3.jpeg)

To switch a device on or off, access the web interface for the Web-I controller through the network. The Web-I will have been assigned an IP address and port during the Sierra Wireless setup. On *Alma*, the Web-I is assigned IP 192.168.13.5 and accessed through public port 1930. The Web-I does not get turned off, and will always show up in the IP table on ACEmanager.

To log in to to the Web-I remotely, enter the modem WAN IP address and port into your browser (166.130.161.5:1930). If you have previously logged in during that browser session, you won't need to enter credentials. 

If you have started a new browser session, or are logging in from a new computer, you will be prompted to enter a username and password. These have never been changed from the default, which has a specific log in for the webpage. 
Username: webftp
Password: 1234

The landing page is the relay board. Each board can be expanded to show all of the available relays, which can be switched On or Off individually or the whole bank can be turned On or Off. 

![Alt](https://github.com/WHOIGit/PhytO-ARM/blob/master/website/static/images/Alma4.png)

If a device is added or changed, you can update these through the Configure Menu. From the Configure page, select the Relay Configuration button at the top. From here, you can update the number of banks, add/delete labels, or rename relays. When assigning a new name, be sure not to include spaces in the name and be sure you're naming the correct relay on the correct board.

![Alt](https://github.com/WHOIGit/PhytO-ARM/blob/master/website/static/images/Alma5.png)

While working onboard, you should hear an audible click when a relay is flipped On or Off. Each board also has an indicator light panel on it, which will illumiate the switches that are currently On. If you need to verify that a device is on while connecting remotely, check the IP table in ACEmanager. Be sure to refresh the table if the device does not appear right away, it can take a few cycles to update and display the new device on the network.


Breaker box table? What's where and what they run on?

#### Launching a Deployment
Deploying cables
Zip tie spots
Leave most of the cable coiled in the housing

#### Monitoring & Maintenance
The battery status should be checked daily and entered into the battery log, which includes charge state, net amperage, and loads on the system. Daily checks are meant to prevent a failure if there is an issue with the charging system or one of the batteries. If the batteries are discharging throughout the day without any periods of recharge this may be a sign that the charging setup is not functioning properly. If left discharging for several days without recharge, battery voltage will drop too low to properly power the system and there may be several outages or an inability to remotely log on to any devices onboard.
The BlueSky controller is manually assigned an IP address rather than reserving one on the modem


Daily maintenance
Check blue sky: login info
battery log & important things to note
when to shut down? check weather forcast

Accessing IP cameras
login info & settings
check to make sure it's not stuck!

Adjust timing on RBR profiles to align with IFCB sampling
IFCB runs bead solutions & each sample isn't the exact same length
target is within 1 min
Adjust task scheduler