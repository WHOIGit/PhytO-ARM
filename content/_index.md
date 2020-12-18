---
title: Home
---

# PhytO-ARM
PhytO-ARM is an extensible, open-source platform for multi-sensor, real-time phytoplankton observing, data sharing, and adaptive sampling. It is intended for scientists and aquaculturists that need to monitor and react to dynamic algal bloom conditions.

PhytO-ARM was conceived as a way to support Imaging FlowCytobot ([IFCB](https://mclanelabs.com/imaging-flowcytobot/)) and Environmental Sample Processor ([ESP](https://mclanelabs.com/environmental-sample-processor/)) sensor installations on floating barges for our work studying harmful algal blooms ([HABs](https://hab.whoi.edu). Prototypical systems couple an IFCB, a cellular modem/GPS, and a profiling CTD (conductivity-temperature-depth logger) to provide geo-tagged whole water column descriptions of temperature, salinity, fluorescence and other hydrographic measurements alongside rich descriptions of phytoplankton species diversity and abundance at specific depths (e.g., chlorophyll maximums). The system is also readily adaptable for IFCB installations on piers, ships, and autonomous vehicles. The full range of PhytO-ARM systems - from barge-based profiling systems to pier and ASV installations - are used extensively through the New England HAB Observing Network ([ne-HABON](https://ne-habon.whoi.edu)).

Through support from NOAA's National Sea Grant program we have been migrating PhytO-ARM to [ROS](https://www.ros.org/about-ros/), an open-source robotics middleware system for coordinating networks of hardware devices (e.g., sensors, actuators, etc.). Our project leverages [contributions to ROS](https://bitbucket.org/%7B87aaf5df-f34f-411d-afb2-556206cefa1d%7D/) made by the [Deep Submergence Laboratoy at WHOI](https://www.whoi.edu/groups/DSL/), takes advantage of other open-source contributions to ROS (e.g., [webviz](https://webviz.io)), and adds new capabilities for interactions with IFCBs and profiling winches.

This site describes the ROS PhytO-ARM system including how to build one of your own. Links describe construction and configuration of a range of system complexities, from simple IFCB -cellular modem/GPS systems(ideal for ship-board installations) to more complicated profiling systems that dynamically reposition sensors and respond when HAB cells are detected.

Hardware designs and parts lists are provided at this site and code is available through the project's [github repository](https://github.com/WHOIGit/PhytO-ARM) and shared through the [GPLv3 license](https://www.gnu.org/licenses/gpl-3.0.en.html).