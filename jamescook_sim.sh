#!/bin/bash
# References:
# Port addresses received from Matt Maltby (RRS James Cook), 10 September 2025
# Save and run this example on IFCB or another pi on network to test parsing/network data capture

DEST_IP="[IP OF RPI5]"
INTERVAL=1  # seconds

send_loop() {
    local msg="$1"
    local port="$2"
    while true; do
        echo "Sending to port $port: $msg"
        echo -n "$msg"$'\r\n' | nc -u -w1 "$DEST_IP" "$port"
        sleep "$INTERVAL"
    done
}

send_array_loop() {
    local -n arr=$1  # use nameref for array
    local port="$2"
    while true; do
        for msg in "${arr[@]}"; do
            echo "Sending to port $port: $msg"
            echo -n "$msg"$'\r\n' | nc -u -w1 "$DEST_IP" "$port"
            sleep "$INTERVAL"
        done
    done
}

# Define messages
cnav_port=19002
cnav_msgs="\$JCTEC,10/09/25,13:25:16.753,mvpos, 0,000,021,+00.60,+00.00,+00.00,45910.55921863,43.713016,-60.812275,-15.45,01,05,218.400,10.600,00.000,00.000,214.700,"

ctd_port=19015
ctd_msg="\$PRTAS,JCMES,10/04/17,20:40:00.135,SBE45, 0,24.58400, 0.00168, 0.01650,1498.26600,25.58210,"

surfmet_port=19023
surfmet_msg="\$PRTSA,JCMES,26/10/18,15:45:50.200,surfm, 0,1.494680,0.049700,4.601600,9.319000,17.316000,22.970000,71.270000,1011.426800,222.900000,213.300000,552.700000,549.300000"


# Start background loops
send_array_loop cnav_msgs "$cnav_port" &
send_loop "$ctd_msg" "$ctd_port" &
send_loop "$surfmet_msg" "$surfmet_port" &


# Keep script running
wait