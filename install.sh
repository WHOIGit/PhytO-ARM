#!/bin/bash

# GPS Converter Installation Script

set -e

echo "Installing GPS Data Converter..."

# Create gps user if it doesn't exist
if ! id "gps" &>/dev/null; then
    echo "Creating gps user..."
    sudo useradd -r -s /bin/false -G dialout gps
fi

# Install the converter script
echo "Installing converter script..."
sudo cp gps-converter.py /usr/local/bin/
sudo chmod +x /usr/local/bin/gps-converter.py
sudo chown root:root /usr/local/bin/gps-converter.py

# Install systemd service
echo "Installing systemd service..."
sudo cp gps-converter.service /etc/systemd/system/
sudo chown root:root /etc/systemd/system/gps-converter.service
sudo chmod 644 /etc/systemd/system/gps-converter.service

# Create log directory
sudo mkdir -p /var/log
sudo touch /var/log/gps-converter.log
sudo chown gps:gps /var/log/gps-converter.log

# Set up device permissions (adjust device path as needed)
DEVICE="/dev/ttyUSB0"
if [ -e "$DEVICE" ]; then
    sudo chown root:dialout "$DEVICE"
    sudo chmod 664 "$DEVICE"
    echo "Set permissions on $DEVICE"
else
    echo "Warning: $DEVICE not found. You may need to adjust device path in service file."
fi

# Reload systemd and enable service
sudo systemctl daemon-reload
sudo systemctl enable gps-converter.service

echo ""
echo "Installation complete!"
echo ""
echo "To start the service:"
echo "  sudo systemctl start gps-converter"
echo ""
echo "To check status:"
echo "  sudo systemctl status gps-converter"
echo ""
echo "To view logs:"
echo "  sudo journalctl -u gps-converter -f"
echo "  tail -f /var/log/gps-converter.log"
echo ""
echo "To test GPSD connection:"
echo "  cgps"
echo "  gpsmon"
echo ""
echo "Note: You may need to adjust the device path in /etc/systemd/system/gps-converter.service"
echo "if your GPS device is not at /dev/ttyUSB0"
