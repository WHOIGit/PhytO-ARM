#!/usr/bin/bash

while true; do
    socat -d -d ${SERIAL_DEVICE},b${BAUD_RATE},raw,echo=0 UDP:${UDP_DESTINATION},bind=:12345
    sleep 1  # add a small delay to prevent potential busy looping
done
