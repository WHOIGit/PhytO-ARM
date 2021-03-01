# PhytO-ARM: *Alma* Guide

#### Purpose of the Guide

This guide was created alongside the updates to the raft platform, now known as *Alma*, to enable new users to understand and interact with the platform, instruments, and remote interface. Much of the information covered in this guide builds on other information, skills, and equipment commonly used in the Brosnahan lab. Please refer to documentation for the individual instruments, network components, and additional hardware for more information on their operation.

By no means is this an exhaustive guide, and it should be updated frequently and thoroughly to include any changes made over time to the system. 



#### Sampling Goals

*Alma* has been deployed annually in Salt Pond in Nauset since 2012, supporting IFCB, ESP, and CTD profiling. At various points, three winches have been used in surveys, though the most recent iteration has only two installed at the time of writing. See the guide on assembly for full details on winch installation.



#### Hardware 

##### Intel Windows10 NUC 
The on-board computer is the fundamental control device for the winch systems. The NUC is a compact PC running Windows10 and acts as the control device for all the onboard winches. See the NUC setup guide for more information about the programs and settings needed for deployment.

##### MAC400 Motor
The onboard motor is a high-precision servo motor that controls profile movements and provides motion feedback over Ethernet. More details on the motor, setup, and use can be found in the MAC motor guide.

##### Sierra Wireless Modem/Router
The modem/router device allows for network connections and for proper traffic direction. The SIM card allows the modem to access the cell network to gain internet access, which in turn means the fixed IP address of the SIM card can be used to remotely access network devices from beyond the network. All devices on the Local Area Network (LAN) can be accessed through public ports using the Wide Area Network (WAN). Entering the public IP address of the modem along with the specified port allows the modem to redirect the request to the appropriate device, as shown in the example diagram below.

![Alt](https://github.com/WHOIGit/PhytO-ARM/blob/master/website/static/images/Alma1.png)

The Sierra Wireless modem is the source of your connection to the entire system and makes remote monitoring possible. For further discussion of IP reservations, port forwarding, and other features, refer to the user manual on Observatory, found under Manuals -> sierrawireless -> MP70.

##### Netgear Switches
Switches are largely self-explanatory, plug-and-play, and are the major point of network connections. The Web-I Ethernet cable should be plugged in to the modem directly, but most other connections are through switches. If you run into issues with network connections, check the switches to ensure they are on and that the lights around the device cable are illuminated. If they are off or only one comes on, that’s an indication that the device is off or operating in a restricted capacity.

##### Switchable Relays
There are three onboard relay boards that control the instruments and devices on the raft. Each board has a current limit (30 amps, 10 amps, 5 amps) that controls how much current each device can draw and the boards are all powered off the 12V DC/DC converter. 

The 5 amp board supports eight switches, while the 10 and 30 amp boards both support four switches. There are three terminal block pins on each switch: NC, COM, and ON. If you ever get confused about which is which, there is a small key in the corner of the board indicating which terminal is which. 

![Alt](https://github.com/WHOIGit/PhytO-ARM/blob/master/website/static/images/Alma2.png)

The cable running to the device connects to the ON terminal so that when the relay is on, the connection between power and the device is open. The COM terminal is where power from the DC/DC converter is connected, supplying power to the switch that is only connected when the relay is switched on. While each of the boards run on 12V, any device run off a relay needs a power supply run through the relay connection. The exact voltage required for each device will vary, and it is important to use the correct voltage for each device regardless of what other voltages are used on the board! For example, the winches and RBR power are all 28V, while the IP cameras are all 12V. Despite the difference in required voltages, these devices are all run off the 5 amp board.

To switch a device on or off, access the web interface for the Web-I controller through the network. The Web-I will have been assigned an IP address and port during the Sierra Wireless setup. 

![Alt](https://github.com/WHOIGit/PhytO-ARM/blob/master/website/static/images/Alma3.png)

Web-I login info
Buttons & navigation
Renaming switches
On-board: hearing the click, indicator light
Check SW to see if IP is active/on network

Breaker box table? What's where and what they run on?

Deploying cables
Zip tie spots
Leave most of the cable coiled in the housing


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