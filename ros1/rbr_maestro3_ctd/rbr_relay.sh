#!/usr/bin/bash

# Script for proxying RBR serial data to a UDP socket
# Useful for having a separate device (e.g. a Raspberry Pi) handle the serial communication
# and then forward the data to a host running PhytO-ARM
# See rbr_relay.service for an example systemd service file.

while true; do
    socat -d -d ${SERIAL_DEVICE},b${BAUD_RATE},raw,echo=0 UDP:${UDP_DESTINATION},bind=:12345
    sleep 1  # add a small delay to prevent potential busy looping
done
