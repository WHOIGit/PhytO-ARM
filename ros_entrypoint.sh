#!/bin/bash
set -eo pipefail

source "/opt/ros/${ROS_DISTRO}/setup.bash"
source /app/install/setup.bash

# Allow `ros2` command to be a symlink to this script in order to set up the ROS
# environment and then run the real `ros2`.
if [[ "$(basename "$0")" == "ros2" ]]; then
    set -- "/opt/ros/${ROS_DISTRO}/bin/ros2" "$@"
fi

exec "$@"
