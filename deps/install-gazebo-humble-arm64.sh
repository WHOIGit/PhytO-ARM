#!/bin/bash
set -eo pipefail

# Jammy provides Gazebo Classic on arm64 through the Open Robotics PPA.
install -m 644 /app/deps/gazebo11-non-amd64.asc /usr/share/keyrings/gazebo11-non-amd64.asc
echo 'deb [arch=arm64 signed-by=/usr/share/keyrings/gazebo11-non-amd64.asc] https://ppa.launchpadcontent.net/openrobotics/gazebo11-non-amd64/ubuntu jammy main' \
    > /etc/apt/sources.list.d/gazebo11-non-amd64.list
apt-get update

# The Humble arm64 repository omits these two ROS packages. Install the
# matching release into the standard ROS prefix without replacing its setup.
source "/opt/ros/$ROS_DISTRO/setup.bash"
work=$(mktemp -d)
trap 'rm -rf "$work"' EXIT
vcs import "$work" < /app/deps/ros2-humble-arm64-deps.rosinstall
packages=("$work/gazebo_ros_pkgs/gazebo_ros" "$work/gazebo_ros_pkgs/gazebo_plugins")
rosdep install --from-paths "${packages[@]}" --ignore-src --rosdistro "$ROS_DISTRO" -y \
    -t build -t buildtool -t build_export -t exec

for package in "${packages[@]}"; do
    build="$work/build/$(basename "$package")"
    cmake -S "$package" -B "$build" -DBUILD_TESTING=OFF -DCMAKE_BUILD_TYPE=Release \
        -DCMAKE_INSTALL_PREFIX="/opt/ros/$ROS_DISTRO"
    cmake --build "$build" --parallel "${CMAKE_BUILD_PARALLEL_LEVEL:-2}" --target install
done

sed -i '/^apt:ros-humble-gazebo-plugins$/d; /^apt:ros-humble-gazebo-ros$/d' /tmp/apt-packages.txt
