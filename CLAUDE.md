# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## About PhytO-ARM

PhytO-ARM is a ROS (Robot Operating System) Noetic-based toolkit for oceanographic sensing, primarily integrating Imaging FlowCytobot (IFCB) and other ocean sensors. It coordinates autonomous profiling missions using winch systems and multiple scientific payloads.

## Build and Test Commands

### Docker Build (Recommended)
```bash
# Build container image
docker build --tag whoi/phyto-arm .

# Pull pre-built image
docker pull whoi/phyto-arm:latest
```

### Testing
```bash
# Run unit tests (Docker)
docker run -it --rm whoi/phyto-arm:latest /bin/bash -c "source devel/setup.bash && catkin test phyto_arm"

# Run config validation tests
python3 scripts/config_validation.test.py

# Test individual packages
catkin test aml_ctd
catkin test dli_power_switch
catkin test ifcb
catkin test jvl_motor
catkin test rbr_maestro3_ctd
```

### Running the System
```bash
# Start main PhytO-ARM processes
./phyto-arm start main /configs/config.yaml

# Start arms (in separate terminals)
./phyto-arm start arm_ifcb /configs/config.yaml
./phyto-arm start arm_chanos /configs/config.yaml

# Stop all processes
./phyto-arm stop

# Using tmux for multiple panes
scripts/tmux_run.sh    # Start all in tmux session
scripts/tmux_kill.sh   # Kill session
```

## Architecture Overview

### Core Design Pattern
PhytO-ARM uses a modular architecture with three main component types:

1. **Device nodes** - Hardware drivers (sensors, motors, cameras)
2. **Behavior nodes** - Mission logic and task coordination
3. **System nodes** - Data logging, networking, and monitoring

### Mission Control Architecture
The system uses an event-driven task execution pattern:

- **ArmBase** class provides the core execution loop for all mission types
- Arms implement `get_next_task()` to define mission-specific behaviors  
- **lock_manager** coordinates winch movements to prevent conflicts
- Each arm operates independently but coordinates through the lock manager

### Key Arm Implementations
- **arm_ifcb.py** - IFCB sampling missions with depth profiling
- **arm_chanos.py** - Chanos sensor payload missions
- **ArmBase** - Abstract base class all arms must inherit from

### Winch Coordination
- Central lock manager limits concurrent winch movements (`max_moving_winches`)
- Arms request clearance before moving winches
- Safety checks for depth bounds and speed limits
- Automatic lock cleanup on startup to handle crashes

### Task Execution Flow
```
loop() -> request_clearance() -> get_next_task() -> send_winch_goal() -> task.callback() -> start_next_task() -> loop()
```

## Configuration System

### Config Files
- Main config: `configs/example.yaml` (use as template)
- Schema validation compares against example.yaml structure
- Runtime parameter updates via ROS Parameter Server

### Key Configuration Sections
- `launch_args` - Core system settings (winch enable, logging paths)
- `gps`, `ifcb`, `arm_ifcb`, `arm_chanos` - Device and mission configs
- `lock_manager` - Winch coordination settings
- `network_data_capture` - UDP/TCP data streams (optional)

### Validation
```bash
# Skip validation
./phyto-arm start main config.yaml --skip_validation

# Use custom schema
./phyto-arm start main config.yaml --config_schema custom.yaml
```

## Key ROS Package Structure

### Main Packages
- **phyto_arm** - Core mission control and behavior nodes
- **aml_ctd**, **rbr_maestro3_ctd** - CTD sensor drivers  
- **ifcb** - IFCB instrument interface
- **jvl_motor** - Winch motor control
- **dli_power_switch** - Power management

### Third-Party Dependencies
- **ds_base**, **ds_msgs**, **ds_sensors** - Woods Hole sensor framework
- **ros-rtsp-camera** - IP camera streaming
- **ros-triton-classifier** - Optional ML classification

## Development Workflow

### Development Container
Use `.devcontainer/devcontainer.json` with VS Code Dev Containers extension for consistent development environment.

### Hardware Mocking
For local development without hardware:
```bash
# Mock IFCB arm
docker exec -it phyto-arm start mock_arm_ifcb /configs/config.yaml

# Mock Chanos arm  
docker exec -it phyto-arm start mock_arm_chanos /configs/config.yaml
```

### Adding New Arms
1. Inherit from `ArmBase` in `src/phyto_arm/src/`
2. Implement `get_next_task(self, last_task)` method
3. Create launch file in `src/phyto_arm/launch/`
4. Add configuration section to example.yaml

### Safety Considerations
- All depth movements are bounds-checked against `winch/range/min` and `winch/range/max`
- Speed limits enforced via `winch/max_speed` parameter
- Winch conflicts prevented by lock manager semaphore
- Hardware enable/disable controls in config prevent accidental movements

## Monitoring and Data

### Foxglove Studio Integration
- Connect via ROS Bridge WebSocket on port 9090
- Use message `header.stamp` for accurate time series plots
- SSH tunnel: `ssh -NL 9090:localhost:9090 <host>`

### Data Logging
- Automatic ROS bag recording to `launch_args/rosbag_prefix` location
- Size-limited bag files with incremental naming
- Camera feeds excluded by default (storage optimization)
- Log files available for Foxglove replay

## Hardware Integration Notes

### CTD Configuration
- **AML CTD**: Requires serial configuration via picocom
- **RBR CTD**: Uses UDP proxy via Raspberry Pi (rbr_relay.sh)

### GPS Setup
- Requires gpsd service running on host
- Container needs gpsd socket access (see README deployment checklist)

### Motor Safety
- JVL motors controlled via TCP/IP
- Position and velocity envelope limits configured per deployment
- Emergency stop functionality via stop service calls