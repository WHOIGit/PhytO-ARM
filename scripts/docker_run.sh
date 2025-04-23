#!/bin/bash

# Default config path
CONFIG='./configs/config.yaml'

# Default command
COMMAND='./phyto-arm start main mounted_config.yaml'

# Parse command-line options
while getopts ":bh" opt; do
    case ${opt} in
        b)
            COMMAND='bash'
            ;;
        h)
            echo "Usage: ./scripts/docker_run.sh [-b] <config file path>"
            echo "-h: Help message."
            echo "-b: Open shell into container instead of launching phyto-arm."
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

docker run --rm -it \
    --name phyto-arm \
    --publish 9090:9090/tcp \
    --publish 8098:8098/tcp \
    --publish 12345:12345/udp \
    --volume "$(pwd)"/configs:/app/configs:ro \
    --volume $CONFIG:/app/mounted_config.yaml:ro \
    --volume /home/ifcb/IFCBacquire/Host/Routines:/routines:ro \
    --volume /data:/data \
    --device /dev/ttyS3 \
    whoi/phyto-arm:latest \
    $COMMAND
