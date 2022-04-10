# PhytO-ARM on ROS

This directory contains ROS nodes for the PhytO-ARM project. The supported ROS release is [Noetic Ninjemys][noetic].

  [noetic]: http://wiki.ros.org/noetic


## Starting and Stopping

The `phyto-arm` tool starts all of the ROS nodes and loads the provided configuration file.

    $ ./phyto-arm config/hades.yaml

The ROS nodes will be shut down when the process is terminated. To run nodes in the background (so that you can disconnect from the system, for example), use the `screen` utility:

    $ screen -dmS phyto-arm ./phyto-arm config/hades.yaml

In the future you can shut down the nodes using:

    $ screen -S phyto-arm -X quit

**Note:** All instruments must already be logging data. Some notes on configuring instruments are included below.


## Configuration

Configuration files live in `configs/` and use the YAML format. It is recommended that each deployment get its own configuration.

The entries in the config file are loaded into the ROS [Parameter Server][]. Some parameters can be dynamically changed while the nodes are running:

    $ source devel/setup.bash
    $ rosparam get /conductor/schedule/every
    60
    $ rosparam set /profiler/data_topic /ctd/aml/port3/chloroblue

Be sure to make the corresponding edits to the config file to persist changes beyond the current session.

  [Parameter Server]: https://wiki.ros.org/Parameter%20Server


## Observing with Foxglove Studio

[Foxglove Studio][] is the recommended software for monitoring the PhytO-ARM system. To connect to a live system, use the "Rosbridge (ROS 1 & 2)" connection type.

The Rosbridge node uses TCP port 9090. Ensure this port is forwarded in your router settings *for trusted clients only*, or use SSH port forwarding:

    $ ssh -NL 9090:localhost:9090 hades.hablab.whoi.edu

In Studio, the connection address is `ws://hades.hablab.whoi.edu:9090` if connecting directly or `ws://localhost:9090` if using the SSH tunnel.

  [Foxglove Studio]: https://foxglove.dev

**Tip:** For the most accurate time series plots, configure plot lines to use a message's `header.stamp` rather than its receive time, which is affected by network latency.


## Data Logs

Data is logged to the ROS bag format using the `rosbag record` command. Bag files can also be viewed with Foxglove Studio.

Almost all internal ROS traffic is logged, with the exception of the camera feed (due to storage requirements) and ROS service calls (due to technical limtations).

The config file controls where the bag files are written:

    launch_args:
        rosbag_prefix: /mnt/data/rosbags/phyto-arm_hades

This results in files in the `/mnt/data/rosbags/` directory with names like `phyto-arm_hades_2022-04-01-14-47-11_0.bag`. The name includes the timestamp of when PhytO-ARM was started, plus a numeric counter. A new bag is created, incrementing the counter, when a configured size limit is reached.

A bag file with the suffix `.active` is currently being written, or was left in an incomplete state by a previous session that was terminated abruptly. Such files can sometimes be repaired.


## Overview of Nodes

### PhytO-ARM behavior nodes

These nodes implement the core PhytO-ARM "algorithm" for sampling and are specific to the design of the platform.

  - `conductor`: Orchestrates sampling
    - Subscribes:
      - `/ifcb/in` for IFCB status messages
      - `/profiler` for profile data
      - `/winch/move_to_depth/result` to determine the success of a transit
    - Publishes:
      - `/conductor/state` with the current activity
      - `/winch/move_to_depth/goal` to set a target depth

  - `profiler`: Creates profiles of CTD data during a cast
    - Subscribes:
      - user-chosen topic provided in config
    - Publishes:
      - `/profiler` with *resampled* profile data

  - `web`: Web API for attaching metadata to IFCB bins
    - Subscribes:
      - `/gps/fix` for GPS fixes

  - `winch`: Controls depth using the motor
    - Subscribes:
      - `/ctd/depth` for monitoring depth
      - `/motor/motion` for monitoring motor state
      - `/winch/move_to_depth/goal` for setting the goal depth
      - `/winch/move_to_depth/cancel` for canceling the current goal
    - Publishes:
      - `/winch/move_to_depth/feedback` with progress updates
      - `/winch/move_to_depth/status` with the status of the current goal
      - `/winch/move_to_depth/result` with the result of the goal


### Device driver nodes

These nodes perform lower-level interactions with hardware components. These nodes are designed to be portable to future projects.

  - `camera`: Video stream
    - Publishes:
      - `/camera/image/compressed` with the compressed video feed

  - `ctd`: Driver for the AML or RBR maestro3 CTD
    - Subscribes:
      - `/ctd_comms/in` for receiving data from CTD
    - Publishes:
      - `/ctd` with conductivity, temperature, and pressure data
      - `/ctd/depth` with depth and pressure data
      - `/ctd/aml/derive/xyz` with derived values (for AML only)
      - `/ctd/aml/port#/xyz` with measured values (for AML only)

  - `ctd_comms`: Bridge for CTD serial communications
    - Publishes:
      - `/ctd_comms/in` for messages received from the CTD's serial port
      - `/ctd_comms/out` for messages sent to the CTD's serial port

  - `gps`: Provides GPS fixes to ROS from `gpsd`
    - Publishes:
      - `/gps/fix` and `/gps/extended_fix`

  - `ifcb`: Bridge for the IFCB websocket API
    - Publishes:
      - `/ifcb/in` for messages received from the IFCB
      - `/ifcb/out` for messages sent to the IFCB
    - Services:
      - `/ifcb/command` to send a message to the IFCB
      - `/ifcb/routine` to send a routine to the IFCB

  - `motor`: Driver for the JVL motor
    - Publishes:
      - `/motor/electrical` with electrical registers
      - `/motor/error` with error registers
      - `/motor/motion` with motion registers
    - Services:
      - `/motor/move` to set the motor's velocity
      - `/motor/set_position_envelope` to set position limits
      - `/motor/stop` to stop the motor


### System nodes

These nodes provide functionality for recording data and connecting to other tools like Foxglove Studio.

  - `rosbag_record`: Records ROS traffic to ROS bag files
    - Publishes:
      - `/begin_write` when a new bag file is created

  - `rosbridge_websocket`: Rosbridge server for Foxglove Studio

  - `rosout`: Logging mechanism
    - Subscribes:
      - `/rosout` for log messages from each node
    - Publishes:
      - `/rosout_agg` with copies of all log messages


## Installation

These steps assume that ROS Noetic has been installed already.

    source /opt/ros/noetic/setup.bash

    sudo apt install -y \
        build-essential \
        python3-catkin-tools \
        python3-rosdep \
        python3-vcstool \
        python3-virtualenv

    # Clone source dependencies
    vcs import src < deps.rosinstall

    # Install ROS dependencies
    sudo rosdep init
    rosdep update
    rosdep install --default-yes --from-paths ./src --ignore-src

    # Patch build configuration for cv_bridge
    # https://github.com/ros-perception/vision_opencv/issues/345
    sudo sed -i 's,/usr/include/opencv,/usr/include/opencv4,g' \
        /opt/ros/noetic/share/cv_bridge/cmake/cv_bridgeConfig.cmake

    # Install additional dependencies
    sed 's/#.*//' apt-requirements.txt | envsubst \
        | xargs sudo apt install -y

    # Create Catkin workspace
    catkin init

    # Build
    catkin build phyto_arm

    # Add user to dialout group (takes effect on next login)
    sudo usermod -a -G dialout $USER


### Install as a service

    sudo ln -sf $(pwd)/phyto-arm.service /etc/systemd/system/phyto-arm.service
    sudo systemctl daemon-reload
    sudo systemctl enable phyto-arm
    sudo systemctl start phyto-arm


## Configuring instruments

### AML CTD

Use `picocom` to to talk to the appropriate serial device. Press <kbd>Enter</kbd> first to interrupt any current logging and get a prompt.

    $ picocom -b 115200 /dev/ttyS3
    > set derive depth y
    > set scan dep
    > set derive sv y
    > set scan sound
    > mmonitor
    ...

Press <kbd>Ctrl-A</kbd>, <kbd>Ctrl-X</kbd> to exit `picocom` while the device is logging.

The time must be set to UTC. To sync the clock, you can use:

    (echo; echo "set fulltime $(date -u "+%Y-%m-%d %H:%M:%S")") \
    | picocom -b 115200 /dev/ttyS3


### GPS

GPS tracking is provided via [gpsd][].

  [gpsd]: https://gpsd.gitlab.io/gpsd/index.html

    sudo apt install -y gpsd gpsd-clients

On Ubuntu, edit `/etc/default/gpsd` to configure the GPS device or network source. For example, to listen for UDP packets broadcast on the local network:

    DEVICES="udp://192.168.13.255:22335"

Monitor that GPS updates are being received using `gpsmon`.
