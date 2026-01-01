#!/bin/bash
set -eo pipefail

source /opt/ros/noetic/setup.bash
source /app/ros1/install/setup.bash

if [ -d /hot/ros1/src ] && [ -n "$(ls -A /hot/ros1/src 2>/dev/null)" ]; then
    (
        cd /hot/ros1

        # Check dependencies - fail if any are missing
        if ! rosdep check --from-paths ./src --ignore-src; then
            echo ""
            echo "ERROR: Hot-patch workspace has missing dependencies!"
            echo "Required dependencies are not installed in this container."
            echo ""
            echo "Please rebuild the container with updated dependencies:"
            echo "  1. Run: ./scripts/generate-rosdep-requirements.sh"
            echo "  2. Rebuild: docker build --tag whoi/phyto-arm:latest ."
            exit 1
        fi

        # Build packages in the hot-patch workspace
        [ ! -f .catkin_workspace ] && catkin init
        catkin config --extend /app/ros1/install --no-install --link-devel
        catkin build
    )

    source /hot/ros1/devel/setup.bash
fi

exec "$@"
