#!/bin/bash

# Default command
COMMAND='./phyto-arm start main mounted_config.yaml'

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
    --publish 8080:8080/tcp \
    --publish 9090:9090/tcp \
    --publish 8098:8098/tcp \
    --publish 11311:11311/tcp \
    --publish 12345:12345/udp \
    --mount type=bind,source="$(pwd)"/configs,target=/app/configs,readonly \
    --mount type=bind,source="$CONFIG",target=/app/mounted_config.yaml,readonly \
    --mount type=bind,source=/home/ifcb/IFCBacquire/Host/Routines,target=/routines,readonly \
    --volume /data:/data \
    --device /dev/ttyS3 \
    whoi/phyto-arm:latest \
    "$COMMAND"
