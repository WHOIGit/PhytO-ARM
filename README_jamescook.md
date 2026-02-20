# Deployment Configuration for RRS *James Cook*

Notes from setup of **PhytO-ARM** for shipboard IFCB observing on the RRS *James Cook* during the Fall 2025 AZMP cruise. 

This PhytO-ARM deplopyment configuration is compatible with an IFCB running the stock McLane disk image (Debian 10 OS) or containerized IFCBacquire. PhytO-ARM is installed and runs on a **Raspberry Pi 5 (RPi5)**. GPS data and flow-through SBE and meteorological tower observations from the RRS *James Cook* are written to IFCB hdr files via the `web_node`.

Operation of PhytO-ARM has been streamlined to capture ship data streams and re-publish this data for inclusion in IFCB header files. The script `docker_run.sh` has been updated to launch only the **master node**. 


PhytO-ARM starts automatically on boot of the RPi5 when installed as a `systemd` service. The IFCB does **not** begin sampling until an operator logs into the IFCBacquire WebUI and clicks **Start Acquisition**. Sampling is stopped via **Stop Acquisition** in the WebUI, and other IFCBacquire settings may be modified as usual, independent of PhytO-ARM operation.

If deployed on the RRS *James Cook* with the same UDP ship data streams, few (if any) changes should be required for a successful deployment.

## Table of Contents

- [System Components](#system-components)
- [Raspberry Pi 5 Set Up](#raspberry-pi-5-setup)
- [PhytO-ARM Installation on the RPi5](#phyto-arm-installation-on-the-rpi5)
- [GPS configuration](#gps-configuration)
- [Capturing Ship Data using Network Data Capture](#capturing-ship-data-using-network-data-capture)
- [Troubleshooting Ship Flowthrough Data](#troubleshooting-ship-flowthrough-data)
- [Options for Power Control](#options-for-power-control)
- [Operation](#operation)

---
## System Components

1. IFCB
2. Raspberry Pi 5 (RPi5)
3. Netgear ProSafe 5-Port Gigabit Switch
4. YINLEADER VTUS-2000 Step-Up / Step-Down Voltage Transformer
5. Digital Logger Power Switch (requires 120 VAC)
6. Sierra Wireless Router (optional; local network / remote access)
7. Starlink (optional; pushing data to shore)

---

## Raspberry Pi 5 Setup

The RPi5 is running **Ubuntu 24.04**. The system microSD card (SanDisk 256 GB Extreme PRO®) was flashed using **Raspberry Pi Imager v1.9.4** on macOS.

The following packages need to be installed with apt-get:

- `openssh-server`
- `rsync`
- `git`
- `gpiod`
- `gpsd`
- `gpsd-clients`
- `curl`

Then, enable OpenSSH service:

```bash
sudo systemctl endable ssh
sudo systemctl start ssh
sudo systemctl status ssh
```

And edit `/etc/ssh/sshd_config` to allow RSA key and password authentication:
```bash
sudo nano /etc/ssh/sshd_config
```

Ensure:
```bash
PubkeyAuthentication yes
PasswordAuthentication yes
```
Restart SSH after making changes:
```bash
sudo systemctl restart ssh
```

Operators will need to establish remote access to the RPi if operating/accessing PhytO-ARM off ship. This could be a VPN service like wireguard/tailscale or a remote desktop service like AnyDesk.

To install Tailscale on the RPi:

```bash
curl -fsSL https://tailscale.com/install.sh | sh
sudo tailscale up
tailscale status
```
Tailscale install documentation: https://tailscale.com/docs/install

### RPi5 Network Configuration

Prior to installation aboard the RRS James Cook, obtain and provide the appropriate MAC address to the ship technician so a static IP assignment can be configured in advance:

- Ethernet MAC address if connecting directly to the ship network via ethernet cable.
- WLAN (Wi-Fi) MAC address if connecting to a non-ship router and accessing ship data via Wi-Fi (prefered).

Retrieve MAC addresses with:
```bash
ip link show
```
eth0 → Ethernet MAC address (link/ether)\
wlan0 → Wi-Fi MAC address (link/ether)

---

## PhytO-ARM Installation on the RPi5
1. On the RPi5, install PhytO-ARM with docker:

  First ensure `docker` is installed:
  ```bash
  docker -v
  ```

  If this does not print a version number, install with:
  ```bash
  sudo apt-get update && sudo apt-get install docker.io
  ```

  Running Docker will also require `sudo` unless you add your user to the `docker` group:

  ```bash
  # Adds user 'ifcb' to 'docker' group
  sudo usermod -aG docker ifcb && newgrp docker
  ```

  Finally, pull the `phyto-arm` image. 
  ```bash
  docker pull whoi/phyto-arm:latest
  ```
2. Clone PhytO-ARM on the RPi5
3. Checkout James Cook branch: `checkout -b vhaggans/james-cook-fall-2025`
4. Install PhytO-ARM as a service on the rPi
    
    ```
    sudo ln -sf $(pwd)/phyto-arm.service /etc/systemd/system/phyto-arm.service
    sudo systemctl daemon-reload
    sudo systemctl enable phyto-arm
    sudo systemctl start phyto-arm
    ```
5. In a web browser on the ship's network, go to http://<RPi5_IP>:8098. You should see output like the following returned in your browser window:
```
{"commitHash": "c203cb925f2f9a2792f49589d15834c48a8008c7", "cruiseID": "RRS James Cook", "ifcbLocation": "Deck Lab, main sink", "ifcbWaterSource": "Flow through seawater intake, 6 m below water line (Bornemann Pumps SLH80-40 Hygienic Twin-Screw Pump). Tapped from Deck Lab sink.", "ctdDepth": 6.0, "gpsLatitude": 43.713016667, "gpsLongitude": -60.812275, "gpsSource": "RRS James Cook", "SBE45ModelSN": "SBE 45, SN:0231", "SBE45LastCal": "16 October, 2024", "SBE38ModelSN": "SBE 38, SN:0490", "SBE38LastCal": "03 January, 2024", "ctdTempSBE45_c": 24.584, "ctdTempSBE38_c": 25.5821, "ctdSal_psu": 0.0165, "ctdCond_sm": 0.00168, "ctdSound_mps": 1498.266, "metFlowRate_lmin": 1.49468, "metFluorescence_v": 0.0497, "metTransmissivity_v": 4.6016, "metSurfaceWindSpeed_ms": 9.319, "metSurfaceWindDirection_deg": 17.316, "metSurfaceAirTemp_c": 22.97, "metSurfaceAirHumid_pct": 71.27, "metSurfaceAirPressure_mbar": 1011.4268, "metPortPAR_v": 222.9, "metStarPAR_v": 213.3, "metPortTIR_v": 552.7, "metStarTIR_v": 549.3}
```

## GPS configuration

GPS tracking is provided via [gpsd][].

  [gpsd]: https://gpsd.gitlab.io/gpsd/index.html

With the format of the James Cook NAV data (referenced as JCTEC for the Fall 2025 deployment), a seperate service runs on the RPi5 to convert the ship GPS feed (broadcasted on port 19002) into NMEA format to be digested by GPSD and published inside the PhytO-ARM container. On the RPi5, copy or link `jctec-converter.service` to /etc/systemd/system. Also copy or link `jctec_converter.py` to usr/local/bin. Enable and start `jctec-converter.service`. The converted GPS feed in NMEA will then be sent over port 22336 on the RPi5. 

In a terminal, use a text editor like `vim` or `nano` to edit `/etc/default/gpsd` so that `gspd` listens to the correct port for GPS data. For example, to configure to listen to port 22336:

```
# Default settings for the gpsd init script and the hotplug wrapper.

# Start the gpsd daemon automatically at boot time
START_DAEMON="true"

# Use USB hotplugging to add new USB devices automatically to the daemon
USBAUTO="false"

# Devices gpsd should collect to at boot time.
# They need to be read/writeable, either by user gpsd or the group dialout.
DEVICES="udp://127.0.0.1:22336"

# Other options you want to pass to gpsd
GPSD_OPTIONS=""
```

Monitor that GPS updates are being received using `gpsmon`.

When running in a container, the gpsd service on the host needs to be modified to accept inbound connections from the container. In terminal, use `systemctl edit gpsd.socket` to create an override file:

    # Allow clients to connect to gpsd from Docker.
    # Based on https://stackoverflow.com/q/42240757
    [Socket]
    ListenStream=
    ListenStream=/var/run/gpsd.sock
    ListenStream=0.0.0.0:2947

## Capturing Ship Data using Network Data Capture
PhytO-ARM is also able to capture and parse UDP streams for capturing ship-based data streams. 

Shipboard data flow: Ship --[UDP]--> rPi --[PhytO-ARM network_data_capture]--> topics --[ros]--> webnode --> IFCB .hdr files.

On the Fall 2025 cruise, a list of available UDP streams were provided (see table below). From these, a subset were chosen for ingestion by PhytO-ARM via the network_data_capture node.

### SBE45 (port 19015)

| Column # | Name | Description | Source |
|--------:|------|-------------|--------|
| 1 | Sentence Identifier | `$PRTAS` | |
| 2 | Message Type | JCMES | |
| 3 | Date | Date (MM/DD/YY) | |
| 4 | Time | Time (hh:mm:ss.000) | |
| 5 | Instrument ID | SBE45 | |
| 6 | NULL | 0 | |
| 7 | temp_h | Temperature from SBE45 | SBE45 |
| 8 | cond | Conductivity | SBE45 |
| 9 | salin | Salinity | SBE45 |
|10 | sndspeed | Sound velocity | SBE45 |
|11 | temp_r | Temperature from SBE38 | |

---

### Surfmet (port 19023)

| Column # | Name | Description | Source |
|--------:|------|-------------|--------|
| 1 | Sentence Identifier | `$PRTSA` | |
| 2 | Message Type | JCMES | |
| 3 | Date | Date (DD/MM/YY) | |
| 4 | Time | Time (hh:mm:ss.000) | |
| 5 | Instrument ID | surfm | |
| 6 | NULL | 0 | |
| 7 | flow | Surface water instrument flow rate (l/min) | `$GPXSM` |
| 8 | flou | Surface water fluorescence (V) | `$GPXSM` |
| 9 | trans | Surface water transmissivity (V) | `$GPXSM` |
|10 | speed | Surface wind relative speed (m/s) | `$GPXSM` |
|11 | direct | Surface wind relative direction (°) | `$GPXSM` |
|12 | airtemp | Surface air temperature (°C) | `$GPXSM` |
|13 | humid | Surface air humidity (%) | `$GPXSM` |
|14 | press | Surface air pressure (mbar) | `$GPXSM` |
|15 | ppar | Port side PAR sensor (volt × 10⁻⁵) | `$GPXSM` |
|16 | spar | Starboard side PAR sensor (volt × 10⁻⁵) | `$GPXSM` |
|17 | ptir | Port side TIR sensor (volt × 10⁻⁵) | `$GPXSM` |
|18 | stir | Starboard side TIR sensor (volt × 10⁻⁵) | `$GPXSM` |

---

### GPS (port 19002)

| Column # | Name | Description | Source |
|--------:|------|-------------|--------|
| 1 | Sentence Identifier | `$JCMES` | |
| 2 | Date | Date (MM/DD/YY) | |
| 3 | Time | Time (hh:mm:ss.000) | |
| 4 | Instrument ID | mvpos | |
| 5 | NULL | 0 | |
| 6 | nbseen | Number of satellites seen (not logged) | |
| 7 | nbused | Number of satellites used to compute position fix | `$GPGGA` |
| 8 | hdop | Horizontal Dilution of Precision | `$GPGGA` |
| 9 | vdop | Vertical Dilution of Precision (not logged) | |
|10 | pdop | Position Dilution of Precision (not logged) | |
|11 | gps time | GPS timestamp applied by the GPS unit | `$GPGGA` |
|12 | Latitude | Latitude degrees | `$GPGGA` |
|13 | Longitude | Longitude minutes | `$GPGGA` |
|14 | alt | Height of vessel reference point above sea level (m) | `$GPGGA` |
|15 | perc | Horizontal position code  | `$GPGGA` |
|16 | mode | GNSS quality indicator | `$GPGGA` |
|17 | gndcrs | Course over ground (degrees) | `$GPVTG` |
|18 | gndspeed | Speed over ground (knots) | `$GPVTG` |
|19 | NULL |  | |
|20 | NULL |  | |
|21 | heading | Heading, true degrees | `$GPHDT` |


All three UDP streams may be simulated from script `jamescook_sim.sh`, which can be run on an IFCB (or on any device on the same network as the RPi) to test UDP capture off the ship or when the ship streams are down. 

```bash
chmod u+x jamescook_sim.sh
./jamescook_sim.sh
```
Additionally, the function `udp_regexp_test.py` can be used to check for correct parsing by selected delimiter in the config for the network_data_capture node. Example checking regular expression ',\s*|\s+':

```bash
$ nc -ulp 19015 |python3 udp_regexp_test.py ',\s*|\s+'
> Split result: ['$PRTAS', 'JCMES', '10/04/17', '20:40:00.135', 'SBE45', '0', '24.58400', '0.00168', '0.01650', '1498.26600', '25.58210', '']
```

### Modifying network_data_capture 

The network_data_capture node is configured in the deployment YAML file (e.g., configs/azmp_fall.yaml).

Use this configuration to:

- Change how incoming ship data is parsed
- Add or remove data fields

First, open the deployment config.yaml:

```bash
nano configs/azmp_fall.yaml
```

**How Parsing Works**

```yaml
network_data_capture:
  topics:
```
Each entry defines:

- A network listener (udp or tcp)
- The port it listens on (must match the systemd service configuration)
- The parsing strategy
- Subtopics for specific data fields

**Adding or Modifying Parsed Fields**

To capture a new data field, add a new entry under subtopics for the relevant data field:

```yaml
subtopics:
  ctdDate:
    field_id: 2
    type: "str"
```

Parameters
- field_id — zero-based index of the parsed field
- type — "str" or "float"

Example from ship_ctd:
```yaml
ctdTempSBE45_c:
  field_id: 6
  type: "float"
```

This creates a ROS topic:

`/ship_ctd/ctdTempSBE45_c`

**Important: Update the Web Node Configuration**

If you add a new subtopic and want it to publish to the metadata, you must also update:

```yaml
web:
  field_map:
```

Example:

```yaml
ctdTempSBE45_c:
  topic: /ship_ctd/ctdTempSBE45_c
  topic_field: data
  default: -999.999
```

If this mapping is missing, the topic will NOT appear in the metadata output.

**After Making Changes**

After editing and saving the config.yaml:

1. Restart the PhytO-ARM systemd service: `sudo systemctl restart phyto-arm`

2. Confirm the new data topics are publishing via the webnode: curl http://localhost:8098

**Common Issues**

- Errors in field_id
- Delimiter mismatch (use_regex_delimiter must match format)
- Incorrect data type (float on non-numeric field)
- Port mismatch between config.yaml and systemd service
- Forgetting to update web node field map

--- 

## Troubleshooting Ship Flowthrough Data
If data is not appearing in IFCB .hdr files, identify where the pipeline is failing.

Ship --[UDP]--> RPi5 --[PhytO-ARM network_data_capture]--> topics --[ros]--> webnode --> IFCB .hdr files.

Check web_node output in terminal:

`curl http://localhost:8098`

Alternatively, in a browser, go to url http://[RPi5_IP]:8098

- If defaults (e.g., -999.99) are shown, start with step #1.
- If data is publishing properly, the issue is isolated to the IFCB settings.txt file (skip to step #3).
1. **Can the RPi5 see the shipboard data/is the ship streaming data at the expected ports?**
To test, first stop PhytO-ARM using a RPi5 terminal window `sudo systemctl stop phyto-arm`. Then, listen on the expected port(s): eg. `nc -lup 19015`. If no data is recieved, there is an issue between the RPi5 and the ship or the simulation script. If deploying on the ship, confirm the RPi is properly on the ship network (especially if also using a local network, there could be IP assignment issues. It is best to ask the ship for a static IP assignment). If using `jamescook_sim.sh`, confirm that the simulation script has the proper IP address of the RPi5 and is sending to that address.

Also, verify UDP ports are exposed in docker_run.sh:

```
docker run "${DOCKER_FLAGS[@]}" \
    --name phyto-arm \
    -e NO_VIRTUALENV=1 \
    --publish 8080:8080/tcp \
    --publish 9090:9090/tcp \
    --publish 8098:8098/tcp \
    --publish 12345:12345/udp \
    --publish 19015:19015/udp \
    --publish 19023:19023/udp \
```

2. **Is there an issue with the configuration of the `network_data_capture` node?**
Use `udp_regexp_test.py` to check the parsing configuration and delimeter as set in the network_data_capture node. Also check the field IDs for each subtopic to confirm they match with the parsing output.
```
network_data_capture: #optional.
    stats_interval: 60 #optional. Default is 60 seconds.
    print_stats: false #optional. Default is false. Useful for debugging connections
    topics:
        ship_ctd: # Name of the topic
            connection_type: "udp" # Can be "udp" or "tcp"
            port: 19015 # this is the port your systemD service is pointing to
            parsing_strategy: "delimited" # Can be "json_dict", "json_array", "raw", or "delimited"
            delimiter: ',\s*|\s+' #optional. Only used if parsing_strategy is "delimited"
            use_regex_delimiter: true
            # Field Descriptions: [0] Stream Identifier [$PRTSA], [1] Sentence Identifier [JCMES],
            # [2] Date [DD/MM/YY], [3] Time [hh:mm:ss.000], [4] Instrument ID [SBE45],
            # [5] NULL, [6] temp_h [temperature from SBE45], [7] cond [conductivity],
            # [8] salin [salinity], [9] sndspeed [sound velocity], [10] temp_r [temperature from SBE38]
            subtopics: #optional. If parsing_strategy is not raw, this will parse the data into subtopics
                ctdDate:
                    field_id: 2
                    type: "str"
```
Next, restart PhytO-ARM `sudo systemctl restart phyto-arm`. Use `curl` commands in terminal to compare values of individual fields to data provided through ship reporting systems. 
Example 1 - show all ship data topics published by RPi5 in terminal:
```
curl -s http://localhost:8098 | jq
```
Output:
```
{
  "commitHash": "c203cb925f2f9a2792f49589d15834c48a8008c7",
  "cruiseID": "RRS James Cook",
  "ifcbLocation": "Deck Lab, main sink",
  "ifcbWaterSource": "Flow through seawater intake, 6 m below water line (Bornemann Pumps SLH80-40 Hygienic Twin-Screw Pump). Tapped from Deck Lab sink.",
  "ctdDepth": 6.0,
  "gpsLatitude": 43.713016667,
  "gpsLongitude": -60.812275,
  "gpsSource": "RRS James Cook",
  "SBE45ModelSN": "SBE 45, SN:0231",
  "SBE45LastCal": "16 October, 2024",
  "SBE38ModelSN": "SBE 38, SN:0490",
  "SBE38LastCal": "03 January, 2024",
  "ctdTempSBE45_c": 24.584,
  "ctdTempSBE38_c": 25.5821,
  "ctdSal_psu": 0.0165,
  "ctdCond_sm": 0.00168,
  "ctdSound_mps": 1498.266,
  "metFlowRate_lmin": 1.49468,
  "metFluorescence_v": 0.0497,
  "metTransmissivity_v": 4.6016,
  "metSurfaceWindSpeed_ms": 9.319,
  "metSurfaceWindDirection_deg": 17.316,
  "metSurfaceAirTemp_c": 22.97,
  "metSurfaceAirHumid_pct": 71.27,
  "metSurfaceAirPressure_mbar": 1011.4268,
  "metPortPAR_v": 222.9,
  "metStarPAR_v": 213.3,
  "metPortTIR_v": 552.7,
  "metStarTIR_v": 549.3
}
```
Example 2 - show data from only a single topic:
```
curl -s http://localhost:8098 | jq '{gpsLatitude}'
```
Output:
```
{
  "gpsLatitude": 43.713016667
}
```

3. **Is data publishing to the webnode but not captured in the IFCB .hdr files?**
Open the IFCB Settings.txt file. Confirm the following:
```
GPSFeed:0
PhytoArmDataSource:1:http://<RPi5_IP>:8098
```
Save the Settings.txt file if any changes are made and restart IFCBAcquire. Confirm settings took effect. 

---

## Options for Power Control
_Power control enables remote power cycling of the IFCB and RPi5 (especially helpful when operating these systems from off ship)._
**Note: Digital Logger Switches require 120VAC. Use with a voltage converter if plugged into outlet with 240VAC.**
>
**A.** Power control via bash script on the Raspberry Pi 5 that controls an AC/DC relay from Digital Loggers: https://www.digital-loggers.com/iot2.html
> 
> The DL AC/DC relay unit has 4 outlets. One 'always-on', one 'normally on', and two 'normally off'. It is controlled via GPIO pin 13 on the RPi5 using script `toggle_outlets.sh`. Usage is provided by `./toggle_outlets.sh -h`. Script is intended for remote power cycling of IFCB. Without input arguments, it reverses 'normally on' and 'normally off' outlets for 20s, then restores their prior state.

**OR**

**B.** Power control via the web interface from the Digital Logger Pro Switch: https://www.digital-loggers.com/pro.html

> User's guide found here: https://www.digital-loggers.com/lpc9man.pdf

---

## Operation

### Hardware & Network
1. Power the IFCB and Raspberry Pi 5 using the voltage transformer and Digital Logger.
 - Confirm that "input voltage selector" is set to 220V on the voltage transformer if using a 240V outlet. 
 - Plug in the DL into the 110-120V outlet on the voltage transformer if using the Pro Switch or AC/DC relay. Assign IFCB and RPi5 to an outlet on the DL switch. 
2. Ensure the RPi5 is connected to the ship network and taking a known IP address.
3. Confirm ship UDP data streams are available (e.g., ports 19015, 19023, 19002).

### Start PhytO-ARM
1. On the RPi5, start the PhytO-ARM service:

```bash
    sudo systemctl enable phyto-arm
    sudo systemctl start phyto-arm
```

> Note: `azmp_fall.yaml` was used for the Fall 2025 deployment on the RRS *James Cook* and is currently referenced by `phyto-arm.service`. Operators should only need to modify config files and systemD services to update port assignments following proper install of PhytO-ARM.

2. Confirm PhytO-ARM is running:
```bash
    sudo systemctl status phyto-arm
```

### Verify Ship Data Ingest
1. Confirm ship data is publishing via the web node:
```bash
    curl http://localhost:8098
```
2. If values are not updating, stop PhytO-ARM and verify UDP streams manually:
```bash
    sudo systemctl stop phyto-arm
    nc -lup 19015
```
### Start IFCB Sampling
1. Open the IFCBacquire WebUI.
2. Click **Start Acquisition**.
3. Confirm ship data appears in IFCB `.hdr` files.
![IFCB Dashboard](images_README/ship-data-on-IFCBDashboard.png)

### Stop Operations
- Stop IFCB sampling via the WebUI.

For help, refer to [Troubleshooting Ship Flowthrough Data](#troubleshooting-ship-flowthrough-data).








