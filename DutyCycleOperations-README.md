# PhytO-ARM Duty Cycling & IFCBAcquire Operations

**Last updated:** January 2026  

**Purpose:** Field and bench operation of inverval-based IFCB profiling with IFCB and winch control.


---
## Starting System (Field & Bench)

### 1. Power & Network Check

- Power on:
  - IFCB
  - rPi 5
  - Winch (if profiling)

- Confirm the Pi, IFCB, and winch can ping each other on the local network.

---

### 2. Verify IFCBAcquire Function

1. Open the IFCBAcquire WebUI from your laptop.
2. Confirm:
   - Humidity and temperature are expected (record humidity)
   - AML is streaming (toggle **AUX** if needed)
     - Use `picocom` on the IFCB
     - If AML does not start streaming automatically after toggling AUX, run:
       ```
       set monitor startup y
       ```
3. Run 1–2 test samples to evaluate flow and IFCBAcquire operation.

---

### 3. Open PhytO-ARM WebUI

Navigate to:
```
http://<rPi>:8080/
```

You may still SSH into the Pi or use VS Code for file edits if needed.

---

### 4. Confirm Configuration

Update the **Config** hosted on the WebUI:

- Bench testing:
```yaml
duty_cycle_winch: false
```

- Field deployment:
```yaml
duty_cycle_winch: true
```

> **Note:** Any WebUI config changes apply immediately but do **not** persist after restarting Phyto-ARM.

For permanent changes:
```bash
ssh pi
nano config_name.yaml
sudo systemctl restart phyto-arm
```

---

### 5. Set Duty Cycle Start Time (UTC)

Edit the rPi YAML config and set the start time as applicable:

```yaml
start_time: "2026-01-13 18:27"
```

Restart PhytO-ARM after editing the config:
```bash
sudo systemctl restart phyto-arm
```

---

### 6. Start Duty Cycling & Profiling

From the PhytO-ARM WebUI:

1. Start **main**
2. Start **rosbag**
   - Wait ~10 seconds to confirm no errors
3. Start **arm_duty_cycle**

---

### 7. Confirm Operation

- Check logs:
```
arm_duty_cycle-arm-3.log
```

- SSH onto the rPi to confirm the webnode is publishing:
```bash
curl http://localhost:8098
```

If profiling and winch movement begin at the scheduled time, the system is running correctly.

---

## Using `systemd` with IFCBAcquire

The default setup has IFCBAcquire installed as a systemd service (ifcbacquire.service). This launches the IFCB WebUI on boot at:

```
http://<IFCB IP address>:8000
```

To start or stop the ifcbacquire service manually:

```bash
sudo systemctl start ifcbacquire
sudo systemctl stop ifcbacquire
sudo systemctl restart ifcbacquire
sudo systemctl status ifcbacquire
sudo systemctl enable ifcbacquire
sudo systemctl disable ifcbacquire
sudo journalctl -f -n 100 -u ifcbacquire
```

---

## Running IFCBAcquire Manually (tmux / Testing)

1. Power up the IFCB.
2. Stop and disable the systemd service:
```bash
sudo systemctl stop ifcbacquire
sudo systemctl disable ifcbacquire
```
3. Create a tmux session:
```bash
tmux new -s ifcbacq
```
4. Start IFCBAcquire:
```bash
./run.sh
```
5. Stop the session as normal:
```bash
tmux kill-session -t ifcbacq
```
---

## Emergency Shutdown

### Duty Cycle or Winch Issue

#### Option A: Web Interface (Preferred)

1. Navigate to:
```
http://<rPi>:8080/
```
2. Click the red **STOP** button on `arm_duty_cycle`.
   - This stops system profiling and all winch movement.
3. To fully restart Phyto-ARM:
```bash
sudo systemctl restart phyto-arm
```

> **Note:** Any configuration changes made via VSCode or `nano` require a Phyto-ARM restart to take effect.

---

#### Option B: SSH onto the rPi

```bash
sudo systemctl stop phyto-arm
sudo systemctl restart phyto-arm
```

---

### Stopping IFCBAcquire

- Option A: Stop via the IFCBAcquire WebUI
- Option B:
```bash
sudo systemctl stop ifcbacquire
```


---

## Troubleshooting AML Data

1. Confirm AML is streaming on the IFCB via `picocom`.
2. Confirm the AML relay service is active:
```bash
systemctl status aml_relay.service
```
3. Confirm data on UDP port 12345:
```bash
nc -lup 12345
```
4. Confirm ROS topics:
```bash
docker exec -it phyto-arm bash -c 'source devel/setup.bash; rostopic list'
```
5. Confirm specific AML streams:
```bash
docker exec -it phyto-arm bash -c 'source devel/setup.bash; rostopic echo /arm_duty_cycle/ctd/aml/port2/turbidity'
```
