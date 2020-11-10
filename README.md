# PhytO-ARM on ROS

This directory contains ROS nodes for the PhytO-ARM project. The supported ROS release is [Melodic Morenia][melodic].

[melodic]: http://wiki.ros.org/melodic


## Setup

    sudo apt install -y \
        python-catkin-tools \
        python-rosdep \
        python-wstool

    # Clone source dependencies
    wstool init --shallow src deps.rosinstall

    # Install system dependencies
    sudo rosdep init
    rosdep update
    rosdep install --default-yes --from-paths ./src --ignore-src

    # Install ROS package dependencies
    sed 's/#.*//' apt-requirements.txt | envsubst \
        | xargs sudo apt install -y

    # Create Catkin workspace
    catkin init

    # Build
    catkin build pa_base


## Execute

    source devel/setup.bash
    roslaunch pa_base phyto_arm.launch


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
