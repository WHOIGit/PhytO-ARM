#!/bin/bash

# GPIO configuration
CHIP="gpiochip4"
PIN=13
DEFAULT_DURATION=20

# Usage message
usage() {
    echo "Usage: $0 [DURATION]"
    echo ""
    echo "Toggle GPIO${PIN} on ${CHIP}."
    echo "  DURATION = 0    → reverse GPIO state indefinitely"
    echo "  DURATION > 0    → reverse for DURATION seconds, then revert"
    echo "  No argument     → reverse for ${DEFAULT_DURATION} seconds (default)"
    exit 1
}

# Handle -h or --help
if [[ "$1" == "-h" || "$1" == "--help" ]]; then
    usage
fi

# Parse duration input
if [[ -z "$1" ]]; then
    DURATION=$DEFAULT_DURATION
elif [[ "$1" =~ ^[0-9]+$ ]]; then
    DURATION="$1"
else
    echo "Invalid duration: $1"
    usage
fi

LOGFILE="/tmp/gpio_toggle_${PIN}.log"
exec > >(tee -a "$LOGFILE") 2>&1
echo "==== GPIO Toggle Script Run ===="
echo "Timestamp: $(date)"
echo "User input: Duration = $DURATION"
echo "GPIO chip: $CHIP"
echo "GPIO pin: $PIN"
echo "Initial GPIO value: $CURRENT_VALUE"
echo "New GPIO value: $NEW_VALUE"

# Get current GPIO value
CURRENT_VALUE=$(gpioget "$CHIP" "$PIN" 2>/dev/null)
if [[ $? -ne 0 ]]; then
    echo "Error: Failed to read GPIO${PIN} on ${CHIP}"
    exit 1
fi

# Determine new value (toggle)
if [[ "$CURRENT_VALUE" == "0" ]]; then
    NEW_VALUE=1
else
    NEW_VALUE=0
fi

# Set new value
gpioset "$CHIP" "$PIN"="$NEW_VALUE" &
SET_PID=$!
echo "GPIO${PIN} on ${CHIP} set to $NEW_VALUE."

# Handle indefinite toggle
if [[ "$DURATION" -eq 0 ]]; then
    echo "Change applied indefinitely."
    exit 0
fi

# Script file to handle countdown and revert (runs in background)
REVERT_SCRIPT="/tmp/revert_gpio_${PIN}.sh"

cat > "$REVERT_SCRIPT" <<EOF
#!/bin/bash
DURATION=$DURATION
CHIP="$CHIP"
PIN=$PIN
ORIG_VALUE=$CURRENT_VALUE

for (( i=0; i<DURATION; i+=5 )); do
    sleep 5
    LEFT=\$((DURATION - i - 5))
    echo "\$LEFT seconds remaining..." >> "$LOGFILE"
done

gpioset "\$CHIP" "\$PIN"="\$ORIG_VALUE"
echo "GPIO\$PIN reverted to \$ORIG_VALUE." >> "$LOGFILE"
EOF

chmod +x "$REVERT_SCRIPT"

# Launch background toggle process with nohup
nohup "$REVERT_SCRIPT" >> "$LOGFILE" 2>&1 &

echo "Toggle will revert in $DURATION seconds. View progress with:"
echo "  tail -f $LOGFILE"
