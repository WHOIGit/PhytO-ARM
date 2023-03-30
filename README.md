# PhytO-ARM

PhytO-ARM is robot operating system (ROS)-based toolkit for integration of Imaging FlowCytobot ([IFCB][mclane]) and other ocean sensors that are commonly deployed by the [Brosnahan Lab][blab]. The supported ROS release is [Noetic Ninjemys][noetic]. 

There are instructions for assembling a profiling configuration at our accompanying [PhytO-ARM][parm] website. The profiling system consists of an IFCB, an [AML ctd][aml], and a winch built from a [JVL servo motor][jvl] and parts from [Conedrive][cdrive] and [Waterman Industries][wiparts]. Once assembled, code provided here enables a variety of continuous sampling behaviors including continuous CTD profiling and targeted collection of IFCB samples at chlorophyll maxima. 

PhytO-ARM observations of location, depth, water properties, etc. are written to IFCB metadata files at time of collection for distribution through an existing webservices system called [IFCB dashboard][ifcbdb]. Additional scripts are provided for export to [CF-compliant NetCDF][cfnet] data formats and upload to repositories like the [NOAA ERDDAP][nerddap] system.

During and after deployments, system operators can review system data using standard ROS tools like [FoxGlove Studio][fglove].  

  [mclane]: https://mclanelabs.com/imaging-flowcytobot/
  [blab]: https://www2.whoi.edu/site/brosnahanlab/
  [noetic]: http://wiki.ros.org/noetic
  [parm]: https://phyto-arm.whoi.edu
  [aml]: https://amloceanographic.com/instrumentation
  [jvl]: https://www.jvl.dk/703/mac400-integrated-servo-motor
  [cdrive]: https://conedrive.com/products/products-2-2/gear-sets-materials-specials-etc-2/
  [wiparts]: https://hiliner.com/product-catalog/deep-water-reels/waterman-industries-deep-drop-reels-replacement-spools-and-spring-arms/
  [cfnet]: http://cfconventions.org
  [nerddap]: https://www.ncei.noaa.gov/erddap/index.html
  [fglove]: https://foxglove.dev/studio
  

## Contents

Three types of ROS nodes are provided. Device nodes enable integration of IFCB, CTD, GPS, servo, and IP camera systems. Behavior nodes interpret device node data streams and direct changes in system behavior. System nodes log ROS traffic and make it available for real-time interactive display (Rosbridge). 


## Installation

These instructions are for install of nodes on an IFCB running Debian 10 ("Buster") OS. Additional instructions are provided for installation as dockerized components that may be distributed across several systems on a network.

These steps assume that ROS Noetic has been installed already.

    source /opt/ros/noetic/setup.bash

    # Install apt package dependencies
    sed 's/#.*//' deps/apt-requirements.txt | envsubst \
        | xargs sudo apt install -y

    # Clone source dependencies
    vcs import src < deps/deps.rosinstall

    # Install ROS dependencies
    sudo rosdep init
    rosdep update
    rosdep install --default-yes --from-paths ./src --ignore-src

    # Patch build configuration for cv_bridge
    # https://github.com/ros-perception/vision_opencv/issues/345
    sudo sed -i 's,/usr/include/opencv,/usr/include/opencv4,g' \
        /opt/ros/noetic/share/cv_bridge/cmake/cv_bridgeConfig.cmake

    # Create Python virtual environment
    python3 -m virtualenv .venv --system-site-packages
    . venv/bin/activate
    python3 -m pip install -r deps/python3-requirements.txt

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


### Install with Docker

This is a work in progress.

Container images are built for `x86_64` and `aarch64` and published automatically on [Docker Hub][hub] by the continuous integration system. The container image can also be built with

    docker build --tag whoi/phyto-arm .

  [hub]: https://hub.docker.com/repository/docker/whoi/phyto-arm

Running the container looks like:
```bash
docker run --rm -it \
    # Name of the container in docker
    --name phyto-arm \
    # Bind TCP port host:container
    --publish 9090:9090/tcp \
    # Bind config directory host:container. ':ro' = read-only
    --volume "$(pwd)"/configs:/configs:ro \
    # Bind IFCBAcquire routines directory host:container
    --volume /home/ifcb/IFCBacquire/Host/Routines:/routines:ro \
    # host:container bind for saving files to host, in this case
    # for the log_dir and rosbag_prefix settings in example.yaml
    --volume /mnt/data:/mnt/data \
    # Give access to CTD device
    --device /dev/ttyS3 \
    # Name of Docker image:tag to use
    whoi/phyto-arm:latest \
    # Start command. Change to use desired config file, should be
    # in the container config directory mapped above.
    ./phyto-arm start /configs/hades_docker.yaml
```

The above can be saved to a bash script for repeatability, commonly `run.sh`.


Each serial device defined in the config file (e.g., for the CTD) must be passed to the container with `--device`.

Any network service running on the host to which a node connects must be changed to refer to the Docker host's IP address of `172.17.0.1`, such as in the params `/gps/host` and `/ifcb/address`.

The gpsd service on the host needs to be modified to accept inbound connections from the container. Use `systemctl edit gpsd.socket` to create an override file:

    # Allow clients to connect to gpsd from Docker.
    # Based on https://stackoverflow.com/q/42240757
    [Socket]
    ListenStream=
    ListenStream=/var/run/gpsd.sock
    ListenStream=0.0.0.0:2947


## Node Inventory

### Behavior nodes

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
      - `/profiler/downcast` as above, but only downcasts
      - `/profiler/downcast` as above, but only upcasts

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


### Device nodes (drivers)

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
      - `/ifcb/image` for full-frame images
      - `/ifcb/roi/image` for detected ROI images only
      - `/ifcb/roi/markers` with rectangular bounds of ROIs
    - Services:
      - `/ifcb/command` to send a message to the IFCB
      - `/ifcb/routine` to send a routine to the IFCB

  - `motor`: Driver for the JVL motor
    - Publishes:
      - `/motor/electrical` with electrical registers
      - `/motor/error` with error registers
      - `/motor/motion` with motion registers
    - Services:
      - `/motor/set_position` to set the motor's position
      - `/motor/set_position_envelope` to set position limits
      - `/motor/set_velocity` to set the motor's velocity
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


## Configuration

Configuration files live in `configs/` and use the YAML format. It is recommended that each deployment get its own configuration.

The entries in the config file are loaded into the ROS [Parameter Server][]. Some parameters can be dynamically changed while the nodes are running:

    $ source devel/setup.bash
    $ rosparam get /conductor/schedule/every
    60
    $ rosparam set /profiler/data_topic /ctd/aml/port3/chloroblue

Be sure to make the corresponding edits to the config file to persist changes beyond the current session.

  [Parameter Server]: https://wiki.ros.org/Parameter%20Server


## Instrument-specific configurations

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


## Starting and Stopping

The `phyto-arm` tool starts all of the ROS nodes and loads the provided configuration file.

    $ ./phyto-arm start config/hades.yaml
    $ ./phyto-arm stop


The ROS nodes will be run in the background (so that you can disconnect from the system, for example) within a `screen` session. This session can be attached with the convenience command

    $ ./phyto-arm attach

Standard `screen` key shortcuts apply, such as using <kbd>Ctrl-A</kbd>, <kbd>D</kbd> to detach again.

**Note:** All instruments must already be logging data. Some notes on configuring instruments are included below.



## Observing with Foxglove Studio

[Foxglove Studio][] is the recommended software for monitoring the PhytO-ARM system. To connect to a live system, use the "Foxglove WebSocket" connection type.

The Foxglove Bridge node uses TCP port 8765. Ensure this port is forwarded in your router settings *for trusted clients only*, or use SSH port forwarding:

    $ ssh -NL 8765:localhost:8765 hades.hablab.whoi.edu

In Studio, the connection address is `ws://hades.hablab.whoi.edu:8765` if connecting directly or `ws://localhost:8765` if using the SSH tunnel.

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


