#!/bin/bash
set -eo pipefail

source "/opt/ros/${ROS_DISTRO}/setup.bash"
source /app/ros2/install/setup.bash

exec "$@"
