#!/bin/bash
set -eo pipefail

source /opt/ros/humble/setup.bash
source /app/ros2/install/setup.bash 2>/dev/null || true

if [ -d /hot/ros2/src ] && [ -n "$(ls -A /hot/ros2/src 2>/dev/null)" ]; then
    (
        cd /hot/ros2

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
        source /opt/ros/humble/setup.bash
        source /app/ros2/install/setup.bash 2>/dev/null || true
        colcon build --symlink-install
    )

    source /hot/ros2/install/setup.bash
fi

exec "$@"
