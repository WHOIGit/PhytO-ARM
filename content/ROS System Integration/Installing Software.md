---
title: "Installing Software"
weight: 1
editor_options: 
  markdown: 
    wrap: 72
---

DRAFT

To make this system work you first need to install various softwares to
your computer and IFCB. Your computer will need to have [FoxGlove Studio
installed](https://foxglove.dev/download). You will need to flash your
IFCB with the containerized version of Phyto-Arm, instructions for this
can be found in the
[Intallation](https://github.com/WHOIGit/PhytO-ARM#installation) section
of the Git Repo pasted below.

## Installation

These steps assume that ROS Noetic has been installed already.

    source /opt/ros/noetic/setup.bash

    # Install apt package dependencies
    sed 's/#.*//' apt-requirements.txt | envsubst \
        | xargs sudo apt install -y

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

    # Create Python virtual environment
    python3 -m virtualenv .venv --system-site-packages
    . venv/bin/activate
    python3 -m pip install -r python3-requirements.txt

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

Container images are built for `x86_64` and `aarch64` and published
automatically on [Docker
Hub](https://hub.docker.com/repository/docker/whoi/phyto-arm) by the
continuous integration system. The container image can also be built
with

    docker build --tag whoi/phyto-arm .

Running the container looks like:

    docker run --rm -it \
        --name phyto-arm \
        --publish 9090:9090/tcp \
        --volume "$(pwd)"/configs:/configs:ro \
        --volume ~/IFCBacquire:/root/IFCBacquire:ro \
        --volume /mnt/data:/mnt/data \
        --device /dev/ttyS3 \
        whoi-phyto-arm \
        ./phyto-arm start /configs/hades_docker.yaml

Each serial device defined in the config file (e.g., for the CTD) must
be passed to the container with `--device`.

Any network service running on the host to which a node connects must be
changed to refer to the Docker host's IP address of `172.17.0.1`, such
as in the params `/gps/host` and `/ifcb/address`.

The gpsd service on the host needs to be modified to accept inbound
connections from the container. Use `systemctl edit gpsd.socket` to
create an override file:

    # Allow clients to connect to gpsd from Docker.
    # Based on https://stackoverflow.com/q/42240757
    [Socket]
    ListenStream=
    ListenStream=/var/run/gpsd.sock
    ListenStream=0.0.0.0:2947
