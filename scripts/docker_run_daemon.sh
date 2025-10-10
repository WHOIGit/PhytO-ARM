#!/bin/bash

# Default config path
CONFIG='./configs/config.yaml'

# Parse command-line options
while getopts ":bh" opt; do
    case ${opt} in
        h)
            echo "Usage: ./scripts/docker_run_daemon.sh [-b] <config file path>"
            echo "-h: Help message."
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
fi

DOCKER_FLAGS=()
DOCKER_FLAGS+=("--rm")
# Check if we're running with a TTY available
[ -t 0 ] && DOCKER_FLAGS+=("-it")


echo "Starting PhytO-ARM server with config: $CONFIG"
echo "Web interface will be available at http://localhost:8080"

docker run "${DOCKER_FLAGS[@]}" \
    --name phyto-arm-daemon \
    --publish 8080:8080/tcp \
    --publish 9090:9090/tcp \
    --publish 8098:8098/tcp \
    --publish 11311:11311/tcp \
    --mount type=bind,source="$(pwd)"/configs,target=/app/configs,readonly \
    --mount type=bind,source="$CONFIG",target=/app/mounted_config.yaml,readonly \
    --mount type=bind,source=/home/hablab/routines,target=/routines,readonly \
    --mount type=bind,source="$(pwd)"/src/phyto_arm,target=/app/src/phyto_arm,readonly \
    --mount type=bind,source="$(pwd)"/src/ifcb,target=/app/src/ifcb,readonly \
    --mount type=bind,source="$(pwd)"/src/dli_power_switch,target=/app/src/dli_power_switch,readonly \
    --volume /data:/data \
    whoi/phyto-arm:latest
