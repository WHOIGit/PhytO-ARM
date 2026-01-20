#!/bin/bash
set -eo pipefail

source /opt/ros/humble/setup.bash
source /app/ros2/install/setup.bash

exec "$@"
