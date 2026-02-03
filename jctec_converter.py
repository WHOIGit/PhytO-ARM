#!/usr/bin/env python3
import socket

def nmea_checksum(sentence):
    checksum = 0
    for char in sentence:
        checksum ^= ord(char)
    return f"{checksum:02X}"

def decimal_to_ddmm(decimal_degrees):
    degrees = int(abs(decimal_degrees))
    minutes = (abs(decimal_degrees) - degrees) * 60.0
    return f"{degrees:02d}{minutes:07.4f}"

def convert_jctec(line):
    # Must start with an NMEA-style '$'
    if not line or not line.startswith('$'):
        return []

    # Split into fields (strip leading '$')
    raw_parts = line.strip()[1:].split(',')

    # Normalize: find the 'JCTEC' marker and drop any leading extra fields (e.g., 'PIFM')
    try:
        start = raw_parts.index('JCTEC')
    except ValueError:
        # Not a JCTEC record
        return []

    parts = [p.strip() for p in raw_parts[start:]]  # from 'JCTEC' onward

    # Expect the classic JCTEC layout from here on (same indices as before)
    # indices:
    # 0:'JCTEC', 1:date, 2:time, 3:'mvpos', 4:'0', 5:'000', 6:nbused, 7:hdop,
    # 8,9,10: offsets, 11:lat, 12:lon, 13:alt, 14:mode?, 15:mode,
    # 16:course, 17:speed, 18,19: zeros, 20:heading, (21: maybe empty from trailing comma)
    if len(parts) < 21:
        return []

    # Extract key fields (same positions as your original working script)
    time_str = parts[2][:8]  # hh:mm:ss
    nbused = parts[6]
    hdop = parts[7]
    try:
        latitude = float(parts[11])
        longitude = float(parts[12])
    except ValueError:
        return []
    altitude = parts[13]
    mode = parts[15]
    course = parts[16]
    speed = parts[17]
    heading = parts[20]

    # Format time and position
    utc_time = time_str.replace(':', '')  # hhmmss
    lat_hem = 'N' if latitude >= 0 else 'S'
    lon_hem = 'E' if longitude >= 0 else 'W'
    lat_ddmm = decimal_to_ddmm(latitude)
    lon_ddmm = decimal_to_ddmm(longitude)

    sentences = []

    # GPGGA
    gpgga = f"GPGGA,{utc_time},{lat_ddmm},{lat_hem},{lon_ddmm},{lon_hem},{mode},{nbused},{hdop},{altitude},M,0.0,M,,"
    sentences.append(f"${gpgga}*{nmea_checksum(gpgga)}")

    # GPVTG
    if course and speed:
        gpvtg = f"GPVTG,{course},T,,M,{speed},N,,K,{mode}"
        sentences.append(f"${gpvtg}*{nmea_checksum(gpvtg)}")

    # GPHDT
    if heading:
        gphdt = f"GPHDT,{heading},T"
        sentences.append(f"${gphdt}*{nmea_checksum(gphdt)}")

    return sentences

# Setup sockets
input_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
input_sock.bind(('', 19002))

output_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
output_sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)

# Main loop
while True:
    data, _ = input_sock.recvfrom(1024)
    line = data.decode('ascii', errors='ignore').strip()

    for sentence in convert_jctec(line):
        output_sock.sendto((sentence + '\r\n').encode(), ('localhost', 22336))
