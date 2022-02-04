# PhytO-ARM on ROS

This directory contains ROS nodes for the PhytO-ARM project. The supported ROS release is [Noetic Ninjemys][noetic].

[noetic]: http://wiki.ros.org/noetic


## Setup

    source /opt/ros/noetic/setup.bash

    sudo apt install -y \
        build-essential \
        python3-catkin-tools \
        python3-rosdep \
        python3-vcstool

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


## Execute

    source devel/setup.bash
    roslaunch phyto_arm phyto_arm.launch


## Install

    sudo ln -sf $(pwd)/phyto-arm.service /etc/systemd/system/phyto-arm.service
    sudo systemctl daemon-reload
    sudo systemctl enable phyto-arm
    sudo systemctl start phyto-arm


## Docker

Install [Docker Engine][] and [Docker Compose][], for example:

    sudo apt install -y \
        python3 \
        python3-pip \
        libffi-dev \
        libssl-dev
    curl -fsSL https://get.docker.com | sudo sh
    sudo -H pip3 install docker-compose

[Docker Engine]: https://docs.docker.com/engine/install/
[Docker Compose]: https://docs.docker.com/compose/install/
