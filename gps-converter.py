#!/usr/bin/env python3
"""
GPS Data Converter - Converts custom JCTEC format to NMEA for GPSD
"""

import sys
import time
import logging
from datetime import datetime
import math

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(levelname)s - %(message)s',
    handlers=[
        logging.FileHandler('/var/log/gps-converter.log'),
        logging.StreamHandler()
    ]
)

def calculate_nmea_checksum(sentence):
    """Calculate NMEA checksum for a sentence (without $ and *)"""
    checksum = 0
    for char in sentence:
        checksum ^= ord(char)
    return f"{checksum:02X}"

def parse_jctec_line(line):
    """Parse a line of JCTEC format data"""
    try:
        fields = line.strip().split(',')
        if len(fields) < 15:
            return None
            
        # Extract relevant fields based on your format
        device_id = fields[0]  # JCTEC
        date_str = fields[1]   # 10/09/25
        time_str = fields[2]   # 13:25:16.753
        msg_type = fields[3]   # mvpos
        
        # Assuming latitude/longitude are in these positions based on your sample
        # You may need to adjust these indices based on actual format
        latitude = float(fields[11])   # 43.713016
        longitude = float(fields[12])  # -60.812275
        
        # Parse timestamp
        date_parts = date_str.split('/')
        time_parts = time_str.split(':')
        
        # Assume MM/DD/YY format (adjust if needed)
        month, day, year = int(date_parts[0]), int(date_parts[1]), int(date_parts[2])
        year = 2000 + year if year < 50 else 1900 + year  # Y2K handling
        
        hour = int(time_parts[0])
        minute = int(time_parts[1])
        second = float(time_parts[2])
        
        timestamp = datetime(year, month, day, hour, minute, int(second))
        
        return {
            'latitude': latitude,
            'longitude': longitude,
            'timestamp': timestamp,
            'device_id': device_id
        }
        
    except (ValueError, IndexError) as e:
        logging.warning(f"Failed to parse line: {line.strip()} - {e}")
        return None

def format_coordinate(coord, is_longitude=False):
    """Convert decimal degrees to NMEA format (DDMM.MMMM)"""
    abs_coord = abs(coord)
    degrees = int(abs_coord)
    minutes = (abs_coord - degrees) * 60
    
    if is_longitude:
        return f"{degrees:03d}{minutes:06.3f}"
    else:
        return f"{degrees:02d}{minutes:06.3f}"

def create_gprmc_sentence(data):
    """Create NMEA GPRMC sentence from parsed data"""
    timestamp = data['timestamp']
    
    # Time in HHMMSS format
    time_str = timestamp.strftime("%H%M%S")
    
    # Date in DDMMYY format
    date_str = timestamp.strftime("%d%m%y")
    
    # Latitude
    lat_nmea = format_coordinate(data['latitude'])
    lat_ns = 'N' if data['latitude'] >= 0 else 'S'
    
    # Longitude  
    lon_nmea = format_coordinate(data['longitude'], is_longitude=True)
    lon_ew = 'E' if data['longitude'] >= 0 else 'W'
    
    # Build sentence (without checksum)
    sentence = f"GPRMC,{time_str},A,{lat_nmea},{lat_ns},{lon_nmea},{lon_ew},0.0,0.0,{date_str},0.0,E"
    
    # Add checksum
    checksum = calculate_nmea_checksum(sentence)
    
    return f"${sentence}*{checksum}"

def create_gpgga_sentence(data):
    """Create NMEA GPGGA sentence from parsed data"""
    timestamp = data['timestamp']
    
    # Time in HHMMSS format
    time_str = timestamp.strftime("%H%M%S")
    
    # Latitude
    lat_nmea = format_coordinate(data['latitude'])
    lat_ns = 'N' if data['latitude'] >= 0 else 'S'
    
    # Longitude
    lon_nmea = format_coordinate(data['longitude'], is_longitude=True)
    lon_ew = 'E' if data['longitude'] >= 0 else 'W'
    
    # Build sentence (basic GPS fix, 4 satellites, HDOP=1.0, altitude=0)
    sentence = f"GPGGA,{time_str},{lat_nmea},{lat_ns},{lon_nmea},{lon_ew},1,04,1.0,0.0,M,0.0,M,,"
    
    # Add checksum
    checksum = calculate_nmea_checksum(sentence)
    
    return f"${sentence}*{checksum}"

def main():
    """Main conversion loop"""
    logging.info("GPS converter starting...")
    
    input_device = sys.argv[1] if len(sys.argv) > 1 else "/dev/ttyUSB0"
    
    try:
        # Open input device
        with open(input_device, 'r') as input_file:
            logging.info(f"Reading from {input_device}")
            
            for line in input_file:
                line = line.strip()
                if not line:
                    continue
                    
                # Parse the custom format
                parsed_data = parse_jctec_line(line)
                if parsed_data is None:
                    continue
                
                # Generate NMEA sentences
                gprmc = create_gprmc_sentence(parsed_data)
                gpgga = create_gpgga_sentence(parsed_data)
                
                # Output to stdout (which will be piped to GPSD)
                print(gprmc)
                print(gpgga)
                sys.stdout.flush()
                
                logging.debug(f"Converted: {parsed_data}")
                
    except FileNotFoundError:
        logging.error(f"Input device {input_device} not found")
        sys.exit(1)
    except KeyboardInterrupt:
        logging.info("Converter stopped by user")
    except Exception as e:
        logging.error(f"Unexpected error: {e}")
        sys.exit(1)

if __name__ == "__main__":
    main()
