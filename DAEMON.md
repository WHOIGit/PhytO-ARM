# PhytO-ARM Daemon Mode

This document describes the new daemon-based architecture for PhytO-ARM that replaces the complex bash→tmux→Docker→Python→ROS launch chain with a simpler always-on daemon + web interface.

## Overview

The PhytO-ARM daemon (`phyto_arm_daemon.py`) is a persistent service that:

- **Manages ROS process lifecycle** - Handles roscore, rosbag, and arm launches
- **Provides web interface** - Simple dashboard for status monitoring and process control  
- **Preserves existing functionality** - Config validation, alerts, environment setup
- **Eliminates wrapper complexity** - No more screen/tmux sessions or exec commands

## Architecture

```
Container: phyto-arm-daemon
├── FastAPI web server (port 8080) 
├── Process manager (PhytoARMDaemon)
├── ROS processes (roscore, rosbag, arms)
└── Alert system (Slack notifications)
```

## Usage

### Quick Start

```bash
# Build the image with daemon support
docker build --tag whoi/phyto-arm .

# Run daemon (replaces all tmux/docker_run scripts)
./scripts/docker_run_daemon.sh configs/your_config.yaml

# Access web interface
open http://localhost:8080
```

### Web Interface

The dashboard provides:

- **Process Status Cards** - Real-time status of all ROS processes
- **Start/Stop Controls** - One-click process management
- **Auto-refresh** - Status updates every 5 seconds
- **Process Details** - PIDs, start times, restart counts

Available processes:
- `roscore` - ROS Master communication hub
- `rosbag` - Data logging (records all topics)
- `main` - Core PhytO-ARM systems  
- `arm_ifcb` - IFCB sampling missions
- `arm_chanos` - Chanos sensor payload
- `arm_oleander` - Custom payload missions

### Command Line Interface

The daemon can also be controlled via API:

```bash
# Get status
curl http://localhost:8080/api/status

# Start a process
curl -X POST http://localhost:8080/api/start/roscore

# Stop a process  
curl -X POST http://localhost:8080/api/stop/arm_ifcb
```

## Configuration

The daemon uses the same YAML configuration system as the original phyto-arm script:

```yaml
# Standard PhytO-ARM config sections
launch_args:
  log_dir: /data/logs
  rosbag_prefix: /data/rosbags/phyto-arm
  
# Alert configuration (preserved from original)
alerts:
  - type: slack
    url: https://hooks.slack.com/services/...
    
# All existing device/arm configurations work unchanged
gps: {...}
ifcb: {...}
arm_ifcb: {...}
```

## Process Management

### Startup Order

The daemon manages proper startup sequencing:

1. **Environment Setup** - Catkin workspace, virtual environment
2. **Config Validation** - Schema validation against example.yaml
3. **Process Registry** - Track all managed processes
4. **Background Monitoring** - Health checks and failure detection

### Process Lifecycle

Each process (`PhytoARMProcess`) tracks:
- **State** - stopped, starting, running, stopping, failed
- **Process Info** - PID, start/stop times, exit codes
- **Health Monitoring** - Automatic failure detection
- **Clean Shutdown** - Proper termination handling

### Failure Handling

- **Automatic Detection** - Monitor process health every 5 seconds
- **Alert Integration** - Send Slack alerts on process failures
- **Restart Tracking** - Count restart attempts per process
- **Graceful Cleanup** - Proper process termination on shutdown

## Comparison with Original

| Aspect | Original phyto-arm | Daemon Mode |
|--------|-------------------|-------------|
| **Launch Complexity** | bash→tmux→docker→python→ROS | Single docker run |
| **Process Management** | screen sessions + exec | Native process manager |
| **Monitoring** | Terminal attachment | Web dashboard |
| **Status Visibility** | Manual screen -r | Real-time web interface |
| **Control Interface** | CLI commands | Web buttons + API |
| **Failure Detection** | Manual checking | Automatic monitoring |
| **Log Access** | screen logs | Centralized logging |

## Migration Guide

### From tmux_run.sh

Replace:
```bash
./scripts/tmux_run.sh configs/config.yaml
# Then manually attach/detach from tmux
```

With:
```bash  
./scripts/docker_run_daemon.sh configs/config.yaml
# Access http://localhost:8080 for control
```

### From docker_run.sh + exec

Replace:
```bash
./scripts/docker_run.sh configs/config.yaml
docker exec -it phyto-arm ./phyto-arm start arm_ifcb /configs/config.yaml  
docker exec -it phyto-arm ./phyto-arm start arm_chanos /configs/config.yaml
```

With:
```bash
./scripts/docker_run_daemon.sh configs/config.yaml
# Use web interface to start individual arms
```

## Development

### Running Locally

```bash
# Install daemon dependencies
pip install fastapi uvicorn[standard]

# Run daemon directly (for development)
python3 ./phyto_arm_daemon.py configs/config.yaml
```

### Testing

```bash
# Test daemon in container
./scripts/docker_run_daemon.sh -b configs/test_mocks.yaml

# Inside container, start daemon
python3 ./phyto_arm_daemon.py /app/mounted_config.yaml
```

### Adding New Arms

The daemon automatically detects launch files in `src/phyto_arm/launch/`:

1. Add new launch file: `src/phyto_arm/launch/arm_newpayload.launch`
2. Update `launch_configs` in `phyto_arm_daemon.py`
3. Rebuild container image

## Security Considerations

- **Network Access** - Web interface exposed on port 8080
- **Process Control** - Full ROS process management via web API
- **Config Access** - Read-only access to configuration files
- **Data Volumes** - Write access to log/data directories only

## Troubleshooting

### Common Issues

**Daemon won't start:**
- Check config file path and permissions
- Verify Docker volume mounts
- Review container logs: `docker logs phyto-arm-daemon`

**Web interface not accessible:**
- Confirm port 8080 is exposed and not blocked
- Check daemon is running: `docker ps`
- Test API directly: `curl http://localhost:8080/api/status`

**Process won't start:**
- Check process logs in web interface  
- Verify hardware devices are accessible
- Confirm config file has required sections

### Logs

```bash
# Container logs (includes daemon + all ROS processes)
docker logs -f phyto-arm-daemon

# Individual process logs via web interface
# Or access ROS logs in mounted /data volume
```

## Future Enhancements

Potential improvements to the daemon:

- **Process Logs** - Stream individual process logs to web interface
- **Config Editor** - Web-based configuration file editing
- **Health Metrics** - CPU, memory, disk usage monitoring  
- **Email Alerts** - Additional alert channels beyond Slack
- **API Authentication** - Secure API access for production
- **Process Scheduling** - Cron-like scheduling for maintenance tasks