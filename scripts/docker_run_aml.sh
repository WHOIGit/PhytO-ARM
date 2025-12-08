#!/bin/bash

################################################################################
# Special edition command for running just the AML CTD node on a remote host
################################################################################

# Default command
COMMAND='./phyto-arm start aml_ctd mounted_config.yaml'

# Parse command-line options
while getopts ":bdh" opt; do
    case ${opt} in
        b)
            COMMAND='bash'
            ;;
        d)
            # Use container default (dashboard)
            COMMAND=''
            ;;
        h)
            echo "Usage: ./scripts/docker_run.sh [-b] [-d] <config file path>"
            echo "-h: Help message."
            echo "-b: Open shell into container instead of launching phyto-arm."
            echo "-d: Run in daemon mode (uses container's default command)."
            echo "<config file path>: Defaults to ./configs/config.yaml if none provided."
            exit 1
            ;;
        \?)
            echo "Invalid option: -$OPTARG" >&2
            exit 1
            ;;
    esac
done
shift $((OPTIND -1))

# If an arg config is passed in use that instead
if [ -n "$1" ]; then
    CONFIG="$1"
else
    echo "Error: No config provided" >&2
    exit 1
fi

DOCKER_FLAGS=()
DOCKER_FLAGS+=("--rm")
# Check if we're running with a TTY available
[ -t 0 ] && DOCKER_FLAGS+=("-it")

docker run "${DOCKER_FLAGS[@]}" \
    --name phyto-arm \
    --network host \
    --mount type=bind,source="$(pwd)"/configs,target=/app/configs,readonly \
    --mount type=bind,source="$CONFIG",target=/app/mounted_config.yaml,readonly \
    --volume /data:/data \
    -e ROS_MASTER_URI=http://10.1.10.55:11311 \
    -e ROS_HOSTNAME=10.1.10.110 \
    --device /dev/ttyS1 \
    whoi/phyto-arm:nathan.figueroa-duty-cycle \
    $COMMAND
