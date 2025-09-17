#!/bin/bash

# Default config path
CONFIG='./configs/config.yaml'

# Default mode is daemon
MODE='daemon'

# Parse command-line options
while getopts ":bh" opt; do
    case ${opt} in
        b)
            MODE='bash'
            ;;
        h)
            echo "Usage: ./scripts/docker_run_daemon.sh [-b] <config file path>"
            echo "-h: Help message."
            echo "-b: Open shell into container instead of launching daemon."
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
    CONFIG=$1
fi

# Check if we're running in systemd (no TTY available)
if [ -t 0 ]; then
    DOCKER_FLAGS="--rm -it"
else
    DOCKER_FLAGS="--rm -d"
fi

# Set command based on mode
if [ "$MODE" = "bash" ]; then
    COMMAND="bash"
else
    COMMAND="python3 ./phyto_arm_daemon.py /app/mounted_config.yaml"
fi

echo "Starting PhytO-ARM daemon with config: $CONFIG"
echo "Web interface will be available at http://localhost:8080"

docker run $DOCKER_FLAGS \
    --name phyto-arm-daemon \
    --publish 8080:8080/tcp \
    --publish 9090:9090/tcp \
    --publish 8098:8098/tcp \
    --publish 12345:12345/udp \
    --mount type=bind,source="$(pwd)"/configs,target=/app/configs,readonly \
    --mount type=bind,source=$CONFIG,target=/app/mounted_config.yaml,readonly \
    --mount type=bind,source=/home/embedpi/routines,target=/routines,readonly \
    --mount type=bind,source="$(pwd)"/src/phyto_arm,target=/app/src/phyto_arm,readonly \
    --mount type=bind,source="$(pwd)"/src/ifcb,target=/app/src/ifcb,readonly \
    --mount type=bind,source="$(pwd)"/src/dli_power_switch,target=/app/src/dli_power_switch,readonly \
    --volume /data:/data \
    whoi/phyto-arm:nathan.figueroa-gps-lockout \
    $COMMAND