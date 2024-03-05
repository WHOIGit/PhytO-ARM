#!/bin/bash

# Default config path
CONFIG='./configs/config.yaml'

# Default command
COMMAND='./phyto-arm start main mounted_config.yaml'

# Parse command-line options
while getopts ":b" opt; do
    case ${opt} in
        b)
            COMMAND='bash'
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
    --volume /mnt/data:/mnt/data \
    --device /dev/ttyS3 \
    phyto-arm:latest \
    $COMMAND
