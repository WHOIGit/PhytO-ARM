#!/bin/bash

docker run --rm -it \
      --name phyto-arm \
      --publish 9090:9090/tcp \
      --publish 8098:8098/tcp \
      --publish 1234:1234/tcp \
      --volume "$(pwd)"/configs:/configs:ro \
      --volume /home/ifcb/IFCBacquire/Host/Routines:/routines:ro \
      --volume /mnt/data:/mnt/data \
      --device /dev/ttyS3 \
      whoi/phyto-arm:latest \
      ./phyto-arm start main /configs/config.yaml
