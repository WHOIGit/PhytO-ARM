#!/bin/bash
set -eo pipefail

source /opt/ros/humble/setup.bash
source /app/ros2/install/setup.bash

# Allow `ros2` command to be a symlink to this script in order to set up the ROS
# environment and then run the real `ros2`.
if [[ "$(basename "$0")" == "ros2" ]]; then
    set -- ros2 "$@"
fi

exec "$@"
