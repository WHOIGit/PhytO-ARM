#!/usr/bin/env python3
"""
PhytO-ARM Control Server 2.0 - Modern web interface for managing PhytO-ARM ROS processes

Enhanced version with:
- Tailwind CSS for modern styling
- HTMX for dynamic frontend interactions
- WebSocket support for real-time log streaming
- Live config editing with validation and ROS parameter updates
"""

import asyncio
import json
import logging
import os
import shlex
import subprocess
import sys
import threading
import time
import urllib.request
from contextlib import asynccontextmanager
from datetime import datetime
from enum import Enum
from pathlib import Path
from typing import Dict, Optional, List, Any
import glob

import yaml
from fastapi import FastAPI, HTTPException, WebSocket, WebSocketDisconnect
from fastapi.responses import HTMLResponse, JSONResponse
from pydantic import BaseModel

from scripts.config_validation import validate_config

# Setup logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)


class ProcessState(Enum):
    STOPPED = "stopped"
    STARTING = "starting"
    RUNNING = "running"
    STOPPING = "stopping"
    FAILED = "failed"


class ProcessInfo(BaseModel):
    name: str
    state: ProcessState
    pid: Optional[int] = None
    started_at: Optional[datetime] = None
    stopped_at: Optional[datetime] = None
    exit_code: Optional[int] = None
    restart_count: int = 0


class ConfigValidationResult(BaseModel):
    valid: bool
    errors: List[str] = []
    warnings: List[str] = []


class PhytoARMProcess:
    """Manages a single ROS process (roscore, rosbag, or launch file)"""

    def __init__(self, name: str, command: str, env: Dict[str, str],
                 dont_kill: bool = False, required: bool = True):
        self.name = name
        self.command = command
        self.env = env
        self.dont_kill = dont_kill  # For rosbag - terminate but don't kill
        self.required = required

        self.process: Optional[subprocess.Popen] = None
        self.info = ProcessInfo(name=name, state=ProcessState.STOPPED)

    def start(self) -> bool:
        """Start the process"""
        if self.is_running():
            logger.warning(f"{self.name} is already running")
            return True

        try:
            self.info.state = ProcessState.STARTING
            logger.info(f"Starting {self.name}: {self.command}")

            self.process = subprocess.Popen(
                self.command,
                shell=True,
                env=self.env,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True
            )

            self.info.pid = self.process.pid
            self.info.started_at = datetime.now()
            self.info.state = ProcessState.RUNNING

            logger.info(f"{self.name} started with PID {self.info.pid}")
            return True

        except Exception as e:
            logger.error(f"Failed to start {self.name}: {e}")
            self.info.state = ProcessState.FAILED
            return False

    def stop(self) -> bool:
        """Stop the process"""
        if not self.is_running():
            logger.warning(f"{self.name} is not running")
            return True

        try:
            self.info.state = ProcessState.STOPPING
            logger.info(f"Stopping {self.name} (PID {self.info.pid})")

            self.process.terminate()
            try:
                exit_code = self.process.wait(timeout=5.0)
                self.info.exit_code = exit_code
            except subprocess.TimeoutExpired:
                if self.dont_kill:
                    logger.warning(f"Failed to terminate {self.name}, but refusing to kill")
                else:
                    logger.warning(f"Failed to terminate {self.name}, killing")
                    self.process.kill()
                    exit_code = self.process.wait()
                    self.info.exit_code = exit_code

            self.info.stopped_at = datetime.now()
            self.info.state = ProcessState.STOPPED
            self.process = None

            logger.info(f"{self.name} stopped with exit code {self.info.exit_code}")
            return True

        except Exception as e:
            logger.error(f"Failed to stop {self.name}: {e}")
            self.info.state = ProcessState.FAILED
            return False

    def is_running(self) -> bool:
        """Check if process is currently running"""
        if self.process is None:
            return False

        poll_result = self.process.poll()
        if poll_result is not None:
            # Process has terminated
            self.info.exit_code = poll_result
            self.info.stopped_at = datetime.now()
            self.info.state = ProcessState.STOPPED if poll_result == 0 else ProcessState.FAILED
            self.process = None
            return False

        return True

    def get_info(self) -> ProcessInfo:
        """Get current process info"""
        # Update running state if needed
        self.is_running()
        return self.info


class PhytoARMServer:
    """Main server class that manages all PhytO-ARM processes"""

    def __init__(self, config_file: Optional[str] = None, config_schema: str = None):
        self.config_file = config_file
        self.config_schema = config_schema or "./configs/example.yaml"
        self.config = None
        self.original_config = None  # Store original for reset
        self.env = None
        self.config_loaded = False  # Track if config is properly loaded

        # Process registry
        self.processes: Dict[str, PhytoARMProcess] = {}

        # Available launch configurations (discovered at runtime)
        self.launch_configs = {}

        # Background monitoring
        self._monitor_task = None
        self._shutdown = False

        # WebSocket connections for log streaming
        self.log_connections: List[WebSocket] = []

    async def initialize(self):
        """Initialize the server - load config if provided, setup environment"""
        try:
            if self.config_file and os.path.exists(self.config_file):
                # Load config if provided
                logger.info(f"Loading config file {self.config_file}")
                await self.load_config_from_file(self.config_file)
            else:
                if self.config_file:
                    logger.warning(f"Config file {self.config_file} not found, starting without config")
                else:
                    logger.info("Starting server without config - config must be loaded via web interface")

            # Start monitoring
            self._monitor_task = asyncio.create_task(self._monitor_processes())

            logger.info("PhytO-ARM server initialized successfully")

        except Exception as e:
            logger.error(f"Failed to initialize server: {e}")
            raise

    async def load_config_from_file(self, config_path: str):
        """Load configuration from a file"""
        try:
            # Validate config
            logger.info(f"Validating config file {config_path}")
            if not validate_config(config_path, self.config_schema):
                raise ValueError("Config validation failed")

            # Load config
            with open(config_path, 'r') as f:
                self.config = yaml.safe_load(f)
                self.original_config = yaml.safe_load(yaml.dump(self.config))  # Deep copy

            self.config_file = config_path
            self.config_loaded = True

            # Setup environment
            self.env = self._prep_environment()

            # Discover available launch files
            self._discover_launch_files()

            logger.info(f"Config loaded successfully from {config_path}")

        except Exception as e:
            logger.error(f"Failed to load config from {config_path}: {e}")
            raise

    async def load_config_from_url(self, url: str) -> bool:
        """Load configuration from a URL"""
        try:
            import urllib.request
            import tempfile

            logger.info(f"Downloading config from {url}")

            # Download config
            with urllib.request.urlopen(url) as response:
                config_content = response.read().decode('utf-8')

            # Write to temporary file for validation
            with tempfile.NamedTemporaryFile(mode='w', suffix='.yaml', delete=False) as tmp:
                tmp.write(config_content)
                tmp_path = tmp.name

            try:
                # Validate config
                validation = self.validate_config_content(config_content)
                if not validation.valid:
                    logger.error(f"Config from URL {url} failed validation: {validation.errors}")
                    return False

                # Load the validated config
                self.config = yaml.safe_load(config_content)
                self.original_config = yaml.safe_load(yaml.dump(self.config))  # Deep copy
                self.config_loaded = True

                # Setup environment
                self.env = self._prep_environment()

                # Discover available launch files
                self._discover_launch_files()

                # Store the content for the editor (but don't overwrite local file)
                self._remote_config_content = config_content

                logger.info(f"Config loaded successfully from URL {url}")
                return True

            finally:
                os.unlink(tmp_path)

        except Exception as e:
            logger.error(f"Failed to load config from URL {url}: {e}")
            return False

    def _discover_launch_files(self):
        """Discover available launch files in the phyto_arm package"""
        parent_dir = os.path.dirname(__file__)
        launch_dir = os.path.join(parent_dir, 'src', 'phyto_arm', 'launch')

        if not os.path.exists(launch_dir):
            logger.warning(f"Launch directory not found: {launch_dir}")
            return

        launch_files = glob.glob(os.path.join(launch_dir, '*.launch'))

        for launch_file in launch_files:
            filename = os.path.basename(launch_file)
            name = os.path.splitext(filename)[0]

            # Skip rosbag.launch as it's handled specially
            if name != 'rosbag':
                description = self._extract_launch_description(launch_file)
                self.launch_configs[name] = {
                    'filename': filename,
                    'description': description,
                    'path': launch_file
                }
                logger.info(f"Discovered launch file: {name} -> {filename}")

        logger.info(f"Found {len(self.launch_configs)} launch configurations")

    def _extract_launch_description(self, launch_file_path: str) -> str:
        """Extract description from launch file comments or XML"""
        try:
            with open(launch_file_path, 'r') as f:
                content = f.read()

            # Look for description in XML comments
            import re

            # Try to find description in comment at top of file
            comment_pattern = r'<!--\s*(.*?)\s*-->'
            comments = re.findall(comment_pattern, content, re.DOTALL)

            for comment in comments:
                # Look for common description patterns
                if any(word in comment.lower() for word in ['description', 'purpose', 'launch']):
                    # Clean up the comment
                    desc = comment.strip().replace('\n', ' ').replace('\r', '')
                    desc = ' '.join(desc.split())  # Normalize whitespace
                    if len(desc) > 100:
                        desc = desc[:97] + "..."
                    return desc

            # Try to extract from launch tag or other XML elements
            desc_pattern = r'<launch[^>]*>\s*<!--\s*(.*?)\s*-->'
            match = re.search(desc_pattern, content, re.DOTALL)
            if match:
                desc = match.group(1).strip()
                return ' '.join(desc.split())[:100]

            # Fallback: use filename-based description
            filename = os.path.basename(launch_file_path)
            name = os.path.splitext(filename)[0]
            return f"Launch configuration: {name}"

        except Exception as e:
            logger.warning(f"Failed to extract description from {launch_file_path}: {e}")
            filename = os.path.basename(launch_file_path)
            name = os.path.splitext(filename)[0]
            return f"Launch configuration: {name}"

    def get_process_metadata(self, process_name: str) -> dict:
        """Get metadata for a process including description and type"""
        # Core system processes
        if process_name == "roscore":
            return {
                "description": "ROS Master - Core communication hub",
                "type": "system",
                "category": "core"
            }
        elif process_name == "rosbag":
            return {
                "description": "Data Logging - Records ROS topics to bag files",
                "type": "system",
                "category": "logging"
            }
        # Launch file processes
        elif process_name in self.launch_configs:
            config = self.launch_configs[process_name]
            if isinstance(config, dict):
                return {
                    "description": config.get('description', f"Launch configuration: {process_name}"),
                    "type": "launch",
                    "category": self._categorize_process(process_name),
                    "filename": config.get('filename', ''),
                    "path": config.get('path', '')
                }
            else:
                # Handle legacy string format
                return {
                    "description": f"Launch configuration: {process_name}",
                    "type": "launch",
                    "category": self._categorize_process(process_name),
                    "filename": config
                }
        else:
            return {
                "description": f"Process: {process_name}",
                "type": "unknown",
                "category": "other"
            }

    def _categorize_process(self, process_name: str) -> str:
        """Categorize process based on name patterns"""
        name_lower = process_name.lower()

        if 'arm' in name_lower:
            return "mission"
        elif any(word in name_lower for word in ['mock', 'sim', 'test']):
            return "simulation"
        elif any(word in name_lower for word in ['sensor', 'ctd', 'gps', 'camera']):
            return "sensors"
        elif any(word in name_lower for word in ['main', 'core', 'base']):
            return "core"
        else:
            return "other"

    def _prep_environment(self) -> Dict[str, str]:
        """Prepare ROS environment - adapted from original phyto-arm script"""
        parent_dir = os.path.dirname(__file__)
        setup_dir = os.path.abspath(os.path.join(parent_dir, 'devel'))

        env = os.environ.copy()
        env['_CATKIN_SETUP_DIR'] = setup_dir

        # Build command to load workspace
        command = f'. {shlex.quote(setup_dir)}/setup.sh && env'

        if os.getenv('NO_VIRTUALENV') is None:
            command = f'. {shlex.quote(parent_dir)}/.venv/bin/activate && ' + command

        # Get environment
        env_out = subprocess.check_output(command, shell=True, env=env)
        for line in env_out.rstrip().split(b'\n'):
            var, _, value = line.partition(b'=')
            env[var.decode()] = value.decode()

        # Allow config to override log directory
        log_dir = self.config.get('launch_args', {}).get('log_dir')
        if log_dir:
            env['ROS_LOG_DIR'] = log_dir

        return env

    def _build_roslaunch_command(self, package: str, launchfile: str) -> str:
        """Build roslaunch command with config args"""
        rl_args = [
            'roslaunch',
            '--wait',
            '--required',
            '--skip-log-check',
            package, launchfile
        ]

        rl_args.append(f'config_file:={os.path.abspath(self.config_file)}')

        for launch_arg, value in self.config.get('launch_args', {}).items():
            if launch_arg != 'launch_prefix':  # Handle separately
                rl_args.append(f'{launch_arg}:={value}')

        return ' '.join(shlex.quote(a) for a in rl_args)

    async def start_process(self, process_name: str) -> bool:
        """Start a specific process"""
        # Check if config is loaded before starting processes
        if not self.has_config():
            logger.error("Cannot start processes - no config loaded")
            return False

        if process_name in self.processes:
            return self.processes[process_name].start()

        # Create new process based on name
        if process_name == "roscore":
            process = PhytoARMProcess(
                name="roscore",
                command="roscore",
                env=self.env
            )
        elif process_name == "rosbag":
            command = self._build_roslaunch_command('phyto_arm', 'rosbag.launch')
            process = PhytoARMProcess(
                name="rosbag",
                command=command,
                env=self.env,
                dont_kill=True  # Don't kill rosbag forcefully
            )
        elif process_name in self.launch_configs:
            config = self.launch_configs[process_name]
            launchfile = config['filename'] if isinstance(config, dict) else config
            command = self._build_roslaunch_command('phyto_arm', launchfile)
            process = PhytoARMProcess(
                name=process_name,
                command=command,
                env=self.env
            )
        else:
            logger.error(f"Unknown process: {process_name}")
            return False

        self.processes[process_name] = process
        return process.start()

    async def stop_process(self, process_name: str) -> bool:
        """Stop a specific process"""
        if process_name not in self.processes:
            logger.warning(f"Process {process_name} not found")
            return False

        return self.processes[process_name].stop()

    async def get_status(self) -> Dict[str, ProcessInfo]:
        """Get status of all processes"""
        status = {}
        for name, process in self.processes.items():
            status[name] = process.get_info()
        return status

    def get_available_log_files(self) -> Dict:
        """Get list of available log files"""
        # Get log directory from config
        log_dir = self.config.get('launch_args', {}).get('log_dir') if self.config else None
        if not log_dir:
            log_dir = os.environ.get('ROS_LOG_DIR', '/tmp')

        latest_dir = os.path.join(log_dir, 'latest')

        if not os.path.exists(latest_dir):
            return {"error": f"Log directory not found: {latest_dir}", "files": []}

        try:
            log_files = []
            for filename in os.listdir(latest_dir):
                if filename.endswith('.log'):
                    file_path = os.path.join(latest_dir, filename)
                    file_info = {
                        "name": filename,
                        "path": file_path,
                        "size": os.path.getsize(file_path),
                        "modified": os.path.getmtime(file_path)
                    }
                    log_files.append(file_info)

            # Sort by modification time, most recent first
            log_files.sort(key=lambda x: x["modified"], reverse=True)

            return {"log_dir": latest_dir, "files": log_files}

        except Exception as e:
            return {"error": f"Error reading log directory: {str(e)}", "files": []}

    def get_log_file_content(self, filename: str, max_lines: int = 100) -> Dict:
        """Get content of a specific log file"""
        # Get log directory from config
        log_dir = self.config.get('launch_args', {}).get('log_dir') if self.config else None
        if not log_dir:
            log_dir = os.environ.get('ROS_LOG_DIR', '/tmp')

        file_path = os.path.join(log_dir, 'latest', filename)

        if not os.path.exists(file_path):
            return {"error": f"Log file not found: {filename}", "lines": []}

        try:
            with open(file_path, 'r', encoding='utf-8', errors='ignore') as f:
                lines = f.readlines()
                recent_lines = [line.rstrip() for line in lines[-max_lines:]]

            return {
                "filename": filename,
                "lines": recent_lines,
                "total_lines": len(lines),
                "file_size": os.path.getsize(file_path)
            }
        except Exception as e:
            return {"error": f"Error reading file {filename}: {str(e)}", "lines": []}

    def get_config_content(self) -> str:
        """Get current config file content"""
        try:
            # If we loaded from URL, return that content
            if hasattr(self, '_remote_config_content'):
                return self._remote_config_content

            # Otherwise read from file
            if self.config_file and os.path.exists(self.config_file):
                with open(self.config_file, 'r') as f:
                    return f.read()
            else:
                return ""
        except Exception as e:
            logger.error(f"Failed to read config file: {e}")
            return ""

    def has_config(self) -> bool:
        """Check if a valid config is loaded"""
        return self.config_loaded and self.config is not None

    def validate_config_content(self, content: str) -> ConfigValidationResult:
        """Validate config content"""
        try:
            # Parse YAML
            config_data = yaml.safe_load(content)

            # Write to temporary file for validation
            import tempfile
            with tempfile.NamedTemporaryFile(mode='w', suffix='.yaml', delete=False) as tmp:
                yaml.dump(config_data, tmp)
                tmp_path = tmp.name

            try:
                # Validate using existing validation function
                is_valid = validate_config(tmp_path, self.config_schema)

                if is_valid:
                    return ConfigValidationResult(valid=True)
                else:
                    return ConfigValidationResult(
                        valid=False,
                        errors=["Config validation failed - check logs for details"]
                    )
            finally:
                os.unlink(tmp_path)

        except yaml.YAMLError as e:
            return ConfigValidationResult(
                valid=False,
                errors=[f"YAML syntax error: {str(e)}"]
            )
        except Exception as e:
            return ConfigValidationResult(
                valid=False,
                errors=[f"Validation error: {str(e)}"]
            )

    async def apply_config_changes(self, content: str) -> bool:
        """Apply config changes and update ROS parameters"""
        try:
            # Validate first
            validation = self.validate_config_content(content)
            if not validation.valid:
                return False

            # Parse new config
            new_config = yaml.safe_load(content)

            # Write to config file
            with open(self.config_file, 'w') as f:
                f.write(content)

            # Update internal config
            self.config = new_config

            # TODO: Update ROS parameters if roscore is running
            # This would require rospy or similar ROS client
            logger.info("Config changes applied successfully")
            return True

        except Exception as e:
            logger.error(f"Failed to apply config changes: {e}")
            return False

    def reset_config_to_original(self) -> str:
        """Reset config to original state"""
        try:
            original_content = yaml.dump(self.original_config, default_flow_style=False)
            with open(self.config_file, 'w') as f:
                f.write(original_content)

            self.config = yaml.safe_load(yaml.dump(self.original_config))  # Deep copy
            return original_content
        except Exception as e:
            logger.error(f"Failed to reset config: {e}")
            return ""

    async def _monitor_processes(self):
        """Background task to monitor process health"""
        while not self._shutdown:
            try:
                for name, process in list(self.processes.items()):
                    # Check if process failed
                    if not process.is_running() and process.info.state == ProcessState.FAILED:
                        logger.error(f"Process {name} has failed")

                        # Send alerts if configured
                        await self._send_alerts(name)

                await asyncio.sleep(5)  # Check every 5 seconds

            except Exception as e:
                logger.error(f"Error in process monitor: {e}")
                await asyncio.sleep(5)

    async def _send_alerts(self, process_name: str, test_mode: bool = False):
        """Send alerts when process fails - adapted from original"""
        if not self.config:
            logger.warning("No config loaded, cannot send alerts")
            return

        alerts = self.config.get('alerts', [])
        deployment = self.config.get('name', 'unknown')

        if not alerts:
            logger.info("No alerts configured")
            return

        for alert in alerts:
            if alert.get('type') == 'slack' and alert.get('url'):
                try:
                    if test_mode:
                        message = {
                            'text': f'🧪 *PhytO-ARM Alert Test*\n - Deployment: _{deployment}_\n - Test Time: {datetime.now().strftime("%Y-%m-%d %H:%M:%S")}\n - Status: Alert system is working correctly!'
                        }
                    else:
                        message = {
                            'text': f'*PhytO-ARM process failed*\n - Deployment: _{deployment}_\n - Process: _{process_name}_'
                        }

                    urllib.request.urlopen(
                        alert['url'],
                        json.dumps(message).encode()
                    )

                    if test_mode:
                        logger.info("Test alert sent successfully")
                    else:
                        logger.info(f"Alert sent for {process_name}")
                except Exception as e:
                    logger.error(f"Failed to send alert: {e}")

    async def test_alerts(self) -> dict:
        """Test alert system"""
        if not self.has_config():
            return {"success": False, "message": "No config loaded"}

        alerts = self.config.get('alerts', [])
        if not alerts:
            return {"success": False, "message": "No alerts configured"}

        try:
            await self._send_alerts("test", test_mode=True)
            return {"success": True, "message": "Test alert sent successfully"}
        except Exception as e:
            return {"success": False, "message": f"Failed to send test alert: {str(e)}"}

    async def add_log_connection(self, websocket: WebSocket):
        """Add a WebSocket connection for log streaming"""
        self.log_connections.append(websocket)

    async def remove_log_connection(self, websocket: WebSocket):
        """Remove a WebSocket connection"""
        if websocket in self.log_connections:
            self.log_connections.remove(websocket)

    async def broadcast_log_update(self, log_data: str):
        """Broadcast log updates to all connected WebSocket clients"""
        if self.log_connections:
            disconnected = []
            for websocket in self.log_connections:
                try:
                    await websocket.send_text(log_data)
                except:
                    disconnected.append(websocket)

            # Clean up disconnected clients
            for ws in disconnected:
                await self.remove_log_connection(ws)

    async def shutdown(self):
        """Gracefully shutdown all processes"""
        logger.info("Shutting down PhytO-ARM server")
        self._shutdown = True

        if self._monitor_task:
            self._monitor_task.cancel()

        # Stop all processes in reverse order (roscore last)
        stop_order = ["rosbag"] + [name for name in self.processes if name not in ["roscore", "rosbag"]] + ["roscore"]

        for name in stop_order:
            if name in self.processes:
                await self.stop_process(name)

        logger.info("PhytO-ARM server shutdown complete")


# Global server instance
server: Optional[PhytoARMServer] = None


@asynccontextmanager
async def lifespan(app: FastAPI):
    # Startup
    global server
    config_file = os.environ.get('PHYTO_ARM_CONFIG')  # Optional now
    server = PhytoARMServer(config_file)
    await server.initialize()

    yield

    # Shutdown
    if server:
        await server.shutdown()


# FastAPI app
app = FastAPI(title="PhytO-ARM Control Interface 2.0", version="2.0.0", lifespan=lifespan)


@app.get("/", response_class=HTMLResponse)
async def get_dashboard():
    """Main dashboard page with Tailwind CSS and HTMX"""
    return HTMLResponse(content="""
    <!DOCTYPE html>
    <html lang="en">
    <head>
        <meta charset="UTF-8">
        <meta name="viewport" content="width=device-width, initial-scale=1.0">
        <title>PhytO-ARM Control Dashboard 2.0</title>
        <script src="/static/tailwind-3.4.17.js"></script>
        <script src="/static/htmx-2.0.7.min.js"></script>
        <script src="/static/htmx-ext-ws-2.0.3.js"></script>
        <style>
            .tab-content { display: none; }
            .tab-content.active { display: block; }
            .htmx-indicator { opacity: 0; transition: opacity 200ms ease-in; }
            .htmx-request .htmx-indicator { opacity: 1; }
            .htmx-request.htmx-indicator { opacity: 1; }
        </style>
    </head>
    <body class="bg-gray-100 font-sans">
        <!-- Header -->
        <header class="bg-blue-900 text-white p-6 shadow-lg">
            <div class="container mx-auto flex justify-between items-center">
                <h1 class="text-3xl font-bold flex items-center">
                    <span class="mr-3">🦠</span>
                    PhytO-ARM Control Dashboard 2.0
                </h1>
                <button
                    class="bg-blue-700 hover:bg-blue-600 px-4 py-2 rounded-lg transition-colors flex items-center"
                    onclick="location.reload()"
                >
                    <span class="mr-2">🔄</span>
                    Refresh
                </button>
            </div>
        </header>

        <!-- Main Container -->
        <div class="container mx-auto p-6">
            <!-- Tab Navigation -->
            <nav class="flex space-x-1 bg-white rounded-lg shadow-md p-2 mb-6">
                <button
                    class="tab-btn flex-1 px-6 py-3 rounded-md text-sm font-medium transition-colors bg-blue-600 text-white"
                    onclick="switchTab('processes')"
                    id="processes-tab-btn"
                >
                    🔧 Processes
                </button>
                <button
                    class="tab-btn flex-1 px-6 py-3 rounded-md text-sm font-medium transition-colors text-gray-600 hover:text-gray-900"
                    onclick="switchTab('logs')"
                    id="logs-tab-btn"
                >
                    📄 Logs
                </button>
                <button
                    class="tab-btn flex-1 px-6 py-3 rounded-md text-sm font-medium transition-colors text-gray-600 hover:text-gray-900"
                    onclick="switchTab('config')"
                    id="config-tab-btn"
                >
                    ⚙️ Config
                </button>
            </nav>

            <!-- Tab Contents -->

            <!-- Processes Tab -->
            <div id="processes-tab" class="tab-content active">
                <div
                    id="processes-container"
                    hx-get="/api/processes/render"
                    hx-trigger="load, every 5s"
                    hx-target="#processes-container"
                    hx-swap="innerHTML"
                    class="min-h-96"
                >
                    <div class="flex justify-center items-center h-96">
                        <div class="animate-spin rounded-full h-12 w-12 border-b-2 border-blue-600"></div>
                    </div>
                </div>
            </div>

            <!-- Logs Tab -->
            <div id="logs-tab" class="tab-content">
                <div class="bg-white rounded-lg shadow-md p-6">
                    <div class="flex justify-between items-center mb-4">
                        <h2 class="text-xl font-semibold text-gray-800">Log Files</h2>
                        <button
                            class="bg-blue-600 hover:bg-blue-700 text-white px-4 py-2 rounded-lg transition-colors"
                            hx-get="/api/logs/files"
                            hx-target="#log-files-container"
                            hx-swap="innerHTML"
                        >
                            🔄 Refresh Files
                        </button>
                    </div>

                    <div
                        id="log-files-container"
                        hx-get="/api/logs/files"
                        hx-trigger="load"
                        hx-target="#log-files-container"
                        hx-swap="innerHTML"
                    >
                        <div class="animate-pulse space-y-4">
                            <div class="h-4 bg-gray-200 rounded w-3/4"></div>
                            <div class="h-4 bg-gray-200 rounded w-1/2"></div>
                        </div>
                    </div>

                    <!-- Log Viewer -->
                    <div class="mt-6">
                        <div id="log-viewer" class="bg-gray-900 text-green-400 p-4 rounded-lg font-mono text-sm h-96 overflow-y-auto">
                            <div class="text-gray-500">Select a log file to view its contents</div>
                        </div>
                    </div>
                </div>
            </div>

            <!-- Config Tab -->
            <div id="config-tab" class="tab-content">
                <div class="bg-white rounded-lg shadow-md">
                    <div class="p-6 border-b border-gray-200">
                        <div class="flex justify-between items-center">
                            <h2 class="text-xl font-semibold text-gray-800">Configuration Editor</h2>
                            <div class="flex space-x-2">
                                <button
                                    id="config-reset-btn"
                                    class="bg-gray-600 hover:bg-gray-700 text-white px-4 py-2 rounded-lg transition-colors"
                                    hx-post="/api/config/reset"
                                    hx-target="#config-editor"
                                    hx-swap="innerHTML"
                                    onclick="clearConfigStatus()"
                                >
                                    🔄 Reset
                                </button>
                                <button
                                    id="config-apply-btn"
                                    class="bg-green-600 hover:bg-green-700 text-white px-4 py-2 rounded-lg transition-colors"
                                    onclick="applyConfig()"
                                >
                                    ✅ Apply Changes
                                </button>
                            </div>
                        </div>

                        <!-- URL Loading Section -->
                        <div class="mt-4 p-4 bg-gray-50 rounded-lg">
                            <h3 class="text-sm font-medium text-gray-800 mb-2">Load Config from URL</h3>
                            <div class="flex space-x-2">
                                <input
                                    type="url"
                                    id="config-url-input"
                                    placeholder="https://example.com/config.yaml"
                                    class="flex-1 px-3 py-2 border border-gray-300 rounded-lg text-sm focus:ring-2 focus:ring-blue-500 focus:border-blue-500"
                                >
                                <button
                                    class="bg-blue-600 hover:bg-blue-700 text-white px-4 py-2 rounded-lg transition-colors text-sm font-medium"
                                    onclick="loadConfigFromUrl()"
                                >
                                    📥 Load
                                </button>
                            </div>
                            <div id="url-load-status" class="mt-2 hidden"></div>
                        </div>

                        <!-- Status Message -->
                        <div id="config-status" class="mt-4 hidden"></div>
                    </div>

                    <div class="p-6">
                        <div
                            id="config-editor"
                            hx-get="/api/config/content"
                            hx-trigger="load"
                            hx-target="#config-editor"
                            hx-swap="innerHTML"
                        >
                            <div class="animate-pulse">
                                <div class="h-4 bg-gray-200 rounded w-full mb-2"></div>
                                <div class="h-4 bg-gray-200 rounded w-5/6 mb-2"></div>
                                <div class="h-4 bg-gray-200 rounded w-4/6"></div>
                            </div>
                        </div>
                    </div>
                </div>
            </div>
        </div>

        <script>
            let originalConfigContent = '';
            let currentConfigContent = '';

            function switchTab(tabName) {
                // Hide all tab contents
                document.querySelectorAll('.tab-content').forEach(content => {
                    content.classList.remove('active');
                });

                // Remove active class from all tab buttons
                document.querySelectorAll('.tab-btn').forEach(btn => {
                    btn.classList.remove('bg-blue-600', 'text-white');
                    btn.classList.add('text-gray-600', 'hover:text-gray-900');
                });

                // Show selected tab content
                document.getElementById(tabName + '-tab').classList.add('active');

                // Add active class to clicked tab button
                const activeBtn = document.getElementById(tabName + '-tab-btn');
                activeBtn.classList.remove('text-gray-600', 'hover:text-gray-900');
                activeBtn.classList.add('bg-blue-600', 'text-white');

                // Load logs when switching to logs tab
                if (tabName === 'logs') {
                    htmx.trigger('#log-files-container', 'load');
                }
            }

            function selectLogFile(filename) {
                // Update selected state
                document.querySelectorAll('.log-file-card').forEach(card => {
                    card.classList.remove('ring-2', 'ring-blue-500');
                });
                event.target.closest('.log-file-card').classList.add('ring-2', 'ring-blue-500');

                // Load log content
                htmx.ajax('GET', `/api/logs/content/${filename}?max_lines=200`, '#log-viewer');
            }

            function onConfigContentLoaded(content) {
                originalConfigContent = content;
                currentConfigContent = content;
                clearConfigStatus();
            }

            function onConfigChange() {
                const textarea = document.getElementById('config-textarea');
                if (textarea) {
                    currentConfigContent = textarea.value;
                    updateConfigStatus();
                }
            }

            function updateConfigStatus() {
                const hasChanges = currentConfigContent !== originalConfigContent;
                const statusDiv = document.getElementById('config-status');

                if (hasChanges) {
                    statusDiv.className = 'mt-4 p-4 bg-yellow-50 border border-yellow-200 rounded-lg';
                    statusDiv.innerHTML = '<p class="text-yellow-800"><strong>⚠️ Unsaved changes</strong> - Changes have not been applied to the system</p>';
                    statusDiv.classList.remove('hidden');
                } else {
                    statusDiv.classList.add('hidden');
                }
            }

            function clearConfigStatus() {
                const statusDiv = document.getElementById('config-status');
                statusDiv.classList.add('hidden');
            }

            function applyConfig() {
                const textarea = document.getElementById('config-textarea');
                if (!textarea) return;

                const content = textarea.value;

                // Show loading state
                const statusDiv = document.getElementById('config-status');
                statusDiv.className = 'mt-4 p-4 bg-blue-50 border border-blue-200 rounded-lg';
                statusDiv.innerHTML = '<p class="text-blue-800">🔄 Validating and applying changes...</p>';
                statusDiv.classList.remove('hidden');

                // Apply changes via API
                fetch('/api/config/apply', {
                    method: 'POST',
                    headers: {
                        'Content-Type': 'application/json',
                    },
                    body: JSON.stringify({ content: content })
                })
                .then(response => response.json())
                .then(data => {
                    if (data.success) {
                        originalConfigContent = content;
                        currentConfigContent = content;
                        statusDiv.className = 'mt-4 p-4 bg-green-50 border border-green-200 rounded-lg';
                        statusDiv.innerHTML = '<p class="text-green-800">✅ Configuration applied successfully</p>';
                        setTimeout(clearConfigStatus, 3000);

                        // Refresh processes to show updated state
                        htmx.trigger('#processes-container', 'load');
                    } else {
                        statusDiv.className = 'mt-4 p-4 bg-red-50 border border-red-200 rounded-lg';
                        statusDiv.innerHTML = `<p class="text-red-800"><strong>❌ Validation failed:</strong></p><ul class="list-disc list-inside mt-2">${data.errors.map(err => `<li>${err}</li>`).join('')}</ul>`;
                    }
                })
                .catch(error => {
                    statusDiv.className = 'mt-4 p-4 bg-red-50 border border-red-200 rounded-lg';
                    statusDiv.innerHTML = '<p class="text-red-800">❌ Error applying configuration</p>';
                });
            }

            function loadConfigFromUrl() {
                const urlInput = document.getElementById('config-url-input');
                const statusDiv = document.getElementById('url-load-status');

                if (!urlInput.value.trim()) {
                    statusDiv.className = 'mt-2 p-3 bg-red-50 border border-red-200 rounded-lg';
                    statusDiv.innerHTML = '<p class="text-red-800 text-sm">Please enter a URL</p>';
                    statusDiv.classList.remove('hidden');
                    return;
                }

                // Show loading state
                statusDiv.className = 'mt-2 p-3 bg-blue-50 border border-blue-200 rounded-lg';
                statusDiv.innerHTML = '<p class="text-blue-800 text-sm">🔄 Loading config from URL...</p>';
                statusDiv.classList.remove('hidden');

                // Load config from URL
                fetch('/api/config/load_url', {
                    method: 'POST',
                    headers: {
                        'Content-Type': 'application/json',
                    },
                    body: JSON.stringify({ url: urlInput.value.trim() })
                })
                .then(response => response.json())
                .then(data => {
                    if (data.success) {
                        statusDiv.className = 'mt-2 p-3 bg-green-50 border border-green-200 rounded-lg';
                        statusDiv.innerHTML = '<p class="text-green-800 text-sm">✅ Config loaded successfully</p>';

                        // Reload the config editor to show new content
                        htmx.trigger('#config-editor', 'load');

                        // Clear the input
                        urlInput.value = '';

                        // Hide status after 3 seconds
                        setTimeout(() => statusDiv.classList.add('hidden'), 3000);
                    } else {
                        statusDiv.className = 'mt-2 p-3 bg-red-50 border border-red-200 rounded-lg';
                        statusDiv.innerHTML = `<p class="text-red-800 text-sm">❌ ${data.message}</p>`;
                    }
                })
                .catch(error => {
                    statusDiv.className = 'mt-2 p-3 bg-red-50 border border-red-200 rounded-lg';
                    statusDiv.innerHTML = '<p class="text-red-800 text-sm">❌ Error loading config from URL</p>';
                });
            }
        </script>
    </body>
    </html>
    """)


# Static file serving for JS libraries
@app.get("/static/tailwind-3.4.17.js")
async def get_tailwind():
    with open("server/tailwind-3.4.17.js", "r") as f:
        return HTMLResponse(content=f.read(), media_type="application/javascript")


@app.get("/static/htmx-2.0.7.min.js")
async def get_htmx():
    with open("server/htmx-2.0.7.min.js", "r") as f:
        return HTMLResponse(content=f.read(), media_type="application/javascript")


@app.get("/static/htmx-ext-ws-2.0.3.js")
async def get_htmx_ws():
    with open("server/htmx-ext-ws-2.0.3.js", "r") as f:
        return HTMLResponse(content=f.read(), media_type="application/javascript")


# API Endpoints

@app.get("/api/status")
async def api_status():
    """Get status of all processes"""
    if not server:
        raise HTTPException(status_code=500, detail="Server not initialized")

    status = await server.get_status()
    return {
        "processes": {name: info.dict() for name, info in status.items()},
        "config_loaded": server.has_config()
    }


@app.get("/api/launch_configs")
async def api_launch_configs():
    """Get available launch configurations"""
    if not server:
        raise HTTPException(status_code=500, detail="Server not initialized")

    return server.launch_configs


@app.get("/api/processes/render", response_class=HTMLResponse)
async def api_render_processes():
    """Render processes as HTML for HTMX"""
    if not server:
        raise HTTPException(status_code=500, detail="Server not initialized")

    status = await server.get_status()
    config_loaded = server.has_config()

    # Build HTML for processes
    html_parts = []

    # Add config status banner if no config is loaded
    if not config_loaded:
        html_parts.append('''
        <div class="bg-yellow-50 border border-yellow-200 rounded-lg p-4 mb-6">
            <div class="flex items-center">
                <div class="flex-shrink-0">
                    <svg class="h-5 w-5 text-yellow-400" viewBox="0 0 20 20" fill="currentColor">
                        <path fill-rule="evenodd" d="M8.257 3.099c.765-1.36 2.722-1.36 3.486 0l5.58 9.92c.75 1.334-.213 2.98-1.742 2.98H4.42c-1.53 0-2.493-1.646-1.743-2.98l5.58-9.92zM11 13a1 1 0 11-2 0 1 1 0 012 0zm-1-8a1 1 0 00-1 1v3a1 1 0 002 0V6a1 1 0 00-1-1z" clip-rule="evenodd" />
                    </svg>
                </div>
                <div class="ml-3">
                    <h3 class="text-sm font-medium text-yellow-800">No Configuration Loaded</h3>
                    <p class="text-sm text-yellow-700 mt-1">
                        Load a configuration file via the Config tab before starting processes.
                    </p>
                </div>
            </div>
        </div>
        ''')

    # Add alerts test button
    alerts_disabled = "" if config_loaded else "disabled"
    html_parts.append(f'''
    <div class="mb-6 flex justify-between items-center">
        <h2 class="text-xl font-semibold text-gray-800">Process Control</h2>
        <button
            class="bg-purple-600 hover:bg-purple-700 disabled:bg-gray-400 disabled:cursor-not-allowed text-white px-4 py-2 rounded-lg transition-colors flex items-center"
            hx-post="/api/alerts/test"
            hx-target="#alert-result"
            hx-swap="innerHTML"
            {alerts_disabled}
        >
            <span class="mr-2">🧪</span>
            Test Alerts
        </button>
    </div>
    <div id="alert-result" class="mb-4"></div>
    ''')

    # Get all available processes and their metadata
    all_processes = {}

    # Add core system processes
    for name in ['roscore', 'rosbag']:
        process_info = status.get(name, ProcessInfo(name=name, state=ProcessState.STOPPED))
        metadata = server.get_process_metadata(name)
        all_processes[name] = {
            'info': process_info,
            'metadata': metadata,
            'priority': 1 if name == 'roscore' else 2  # Core processes first
        }

    # Add discovered launch configs
    for name in server.launch_configs.keys():
        process_info = status.get(name, ProcessInfo(name=name, state=ProcessState.STOPPED))
        metadata = server.get_process_metadata(name)
        all_processes[name] = {
            'info': process_info,
            'metadata': metadata,
            'priority': 3  # Launch configs after core
        }

    # Sort processes by priority, then by category, then by name
    def sort_key(item):
        name, data = item
        return (
            data['priority'],
            data['metadata']['category'],
            name
        )

    sorted_processes = sorted(all_processes.items(), key=sort_key)

    # Add processes grid
    html_parts.append('<div class="grid grid-cols-1 md:grid-cols-2 xl:grid-cols-3 gap-6">')

    for name, data in sorted_processes:
        html_parts.append(_render_process_card(data['info'], data['metadata'], config_loaded))

    html_parts.append('</div>')
    return HTMLResponse(content=''.join(html_parts))


def _render_process_card(process_info: ProcessInfo, metadata: dict, config_loaded: bool = True) -> str:
    """Render a single process card"""
    state = process_info.state.value

    # State-specific styling
    state_colors = {
        'running': 'bg-green-100 text-green-800 border-green-200',
        'stopped': 'bg-gray-100 text-gray-800 border-gray-200',
        'starting': 'bg-yellow-100 text-yellow-800 border-yellow-200',
        'stopping': 'bg-orange-100 text-orange-800 border-orange-200',
        'failed': 'bg-red-100 text-red-800 border-red-200'
    }

    # Category-specific styling for card border
    category_colors = {
        'core': 'border-l-4 border-l-blue-500',
        'logging': 'border-l-4 border-l-purple-500',
        'mission': 'border-l-4 border-l-green-500',
        'simulation': 'border-l-4 border-l-yellow-500',
        'sensors': 'border-l-4 border-l-indigo-500',
        'other': 'border-l-4 border-l-gray-400'
    }

    state_class = state_colors.get(state, 'bg-gray-100 text-gray-800 border-gray-200')
    category_class = category_colors.get(metadata.get('category', 'other'), 'border-l-4 border-l-gray-400')
    is_running = state == 'running'
    is_transitioning = state in ['starting', 'stopping']

    # Disable buttons if no config loaded
    buttons_disabled = not config_loaded or is_transitioning
    start_disabled = buttons_disabled or is_running
    stop_disabled = buttons_disabled or not is_running

    # Format timestamps
    started_at = ''
    if process_info.started_at:
        started_at = f'<p class="text-sm text-gray-600 mt-2"><strong>Started:</strong> {process_info.started_at.strftime("%Y-%m-%d %H:%M:%S")}</p>'

    restart_info = ''
    if process_info.restart_count > 0:
        restart_info = f'<p class="text-sm text-gray-600"><strong>Restarts:</strong> {process_info.restart_count}</p>'

    pid_info = ''
    if process_info.pid:
        pid_info = f'<p class="text-sm text-gray-600"><strong>PID:</strong> {process_info.pid}</p>'

    # Add type/category badge
    type_info = ''
    if metadata.get('type') == 'launch' and metadata.get('filename'):
        type_info = f'<p class="text-xs text-gray-500 mt-1">📄 {metadata["filename"]}</p>'

    # Config warning if not loaded
    config_warning = ''
    if not config_loaded:
        config_warning = '<p class="text-xs text-yellow-600 mt-1">⚠️ Config required</p>'

    return f"""
    <div class="bg-white rounded-lg shadow-md p-6 border border-gray-200 {category_class}">
        <div class="flex justify-between items-start mb-4">
            <div>
                <h3 class="text-lg font-semibold text-gray-900">{process_info.name}</h3>
                <p class="text-gray-600 text-sm mt-1">{metadata.get('description', 'No description')}</p>
                {type_info}
                {config_warning}
            </div>
            <div class="flex flex-col items-end space-y-1">
                <span class="px-3 py-1 rounded-full text-xs font-medium border {state_class}">
                    {state.upper()}
                </span>
                <span class="px-2 py-1 bg-gray-100 text-gray-700 rounded text-xs">
                    {metadata.get('category', 'other').title()}
                </span>
            </div>
        </div>

        <div class="space-y-1">
            {pid_info}
            {started_at}
            {restart_info}
        </div>

        <div class="flex space-x-2 mt-4">
            <button
                class="flex-1 bg-green-600 hover:bg-green-700 disabled:bg-gray-400 disabled:cursor-not-allowed text-white px-4 py-2 rounded-lg transition-colors text-sm font-medium"
                hx-post="/api/processes/{process_info.name}/start"
                hx-target="#processes-container"
                hx-swap="innerHTML"
                {"disabled" if start_disabled else ""}
            >
                ▶️ Start
            </button>
            <button
                class="flex-1 bg-red-600 hover:bg-red-700 disabled:bg-gray-400 disabled:cursor-not-allowed text-white px-4 py-2 rounded-lg transition-colors text-sm font-medium"
                hx-post="/api/processes/{process_info.name}/stop"
                hx-target="#processes-container"
                hx-swap="innerHTML"
                {"disabled" if stop_disabled else ""}
            >
                ⏹️ Stop
            </button>
        </div>
    </div>
    """


@app.post("/api/processes/{process_name}/start")
async def api_start_process(process_name: str):
    """Start a specific process"""
    if not server:
        raise HTTPException(status_code=500, detail="Server not initialized")

    success = await server.start_process(process_name)
    if not success:
        raise HTTPException(status_code=500, detail=f"Failed to start {process_name}")

    # Return updated processes HTML
    return await api_render_processes()


@app.post("/api/processes/{process_name}/stop")
async def api_stop_process(process_name: str):
    """Stop a specific process"""
    if not server:
        raise HTTPException(status_code=500, detail="Server not initialized")

    success = await server.stop_process(process_name)
    if not success:
        raise HTTPException(status_code=500, detail=f"Failed to stop {process_name}")

    # Return updated processes HTML
    return await api_render_processes()


@app.get("/api/logs/files", response_class=HTMLResponse)
async def api_get_log_files():
    """Get list of available log files as HTML"""
    if not server:
        raise HTTPException(status_code=500, detail="Server not initialized")

    data = server.get_available_log_files()

    if data.get("error"):
        return HTMLResponse(content=f'<div class="text-red-600 p-4 bg-red-50 rounded-lg">{data["error"]}</div>')

    if not data.get("files"):
        return HTMLResponse(content='<div class="text-gray-600 p-4 bg-gray-50 rounded-lg">No log files found</div>')

    html_parts = ['<div class="grid grid-cols-1 md:grid-cols-2 lg:grid-cols-3 gap-4">']

    for file_info in data["files"]:
        file_size = f"{file_info['size'] / 1024:.1f} KB"
        mod_date = datetime.fromtimestamp(file_info['modified']).strftime("%Y-%m-%d %H:%M:%S")

        html_parts.append(f"""
        <div class="log-file-card bg-white border border-gray-200 rounded-lg p-4 cursor-pointer hover:border-blue-500 hover:shadow-md transition-all"
             onclick="selectLogFile('{file_info['name']}')">
            <h4 class="font-medium text-gray-900 truncate">{file_info['name']}</h4>
            <p class="text-sm text-gray-600 mt-1"><strong>Size:</strong> {file_size}</p>
            <p class="text-sm text-gray-600"><strong>Modified:</strong> {mod_date}</p>
        </div>
        """)

    html_parts.append('</div>')
    return HTMLResponse(content=''.join(html_parts))


@app.get("/api/logs/content/{filename}", response_class=HTMLResponse)
async def api_get_log_content(filename: str, max_lines: int = 200):
    """Get content of a specific log file as HTML"""
    if not server:
        raise HTTPException(status_code=500, detail="Server not initialized")

    data = server.get_log_file_content(filename, max_lines)

    if data.get("error"):
        return HTMLResponse(content=f'<div class="text-red-400">Error: {data["error"]}</div>')

    header = f"""=== {data['filename']} ===
Total lines: {data['total_lines']} | File size: {data['file_size'] / 1024:.1f} KB
Showing last {len(data['lines'])} lines

"""

    content = header + '\n'.join(data['lines'])
    return HTMLResponse(content=f'<pre class="whitespace-pre-wrap">{content}</pre>')


@app.get("/api/config/content", response_class=HTMLResponse)
async def api_get_config_content():
    """Get config file content as HTML textarea"""
    if not server:
        raise HTTPException(status_code=500, detail="Server not initialized")

    content = server.get_config_content()

    # Escape backticks for JavaScript
    escaped_content = content.replace("`", "\\`")

    return HTMLResponse(content=f"""
    <textarea
        id="config-textarea"
        class="w-full h-96 p-4 border border-gray-300 rounded-lg font-mono text-sm resize-none focus:ring-2 focus:ring-blue-500 focus:border-blue-500"
        placeholder="Configuration file content..."
        onchange="onConfigChange()"
        oninput="onConfigChange()"
    >{content}</textarea>
    <script>
        onConfigContentLoaded(`{escaped_content}`);
    </script>
    """)


@app.post("/api/config/apply")
async def api_apply_config(request_data: dict):
    """Apply config changes"""
    if not server:
        raise HTTPException(status_code=500, detail="Server not initialized")

    content = request_data.get('content', '')

    # Validate first
    validation = server.validate_config_content(content)

    if not validation.valid:
        return JSONResponse(content={
            "success": False,
            "errors": validation.errors,
            "warnings": validation.warnings
        })

    # Apply changes
    success = await server.apply_config_changes(content)

    return JSONResponse(content={
        "success": success,
        "errors": [] if success else ["Failed to apply configuration"],
        "warnings": []
    })


@app.post("/api/config/reset", response_class=HTMLResponse)
async def api_reset_config():
    """Reset config to original state"""
    if not server:
        raise HTTPException(status_code=500, detail="Server not initialized")

    content = server.reset_config_to_original()

    # Escape backticks for JavaScript
    escaped_content = content.replace("`", "\\`")

    return HTMLResponse(content=f"""
    <textarea
        id="config-textarea"
        class="w-full h-96 p-4 border border-gray-300 rounded-lg font-mono text-sm resize-none focus:ring-2 focus:ring-blue-500 focus:border-blue-500"
        placeholder="Configuration file content..."
        onchange="onConfigChange()"
        oninput="onConfigChange()"
    >{content}</textarea>
    <script>
        onConfigContentLoaded(`{escaped_content}`);
    </script>
    """)


@app.post("/api/config/load_url")
async def api_load_config_from_url(request_data: dict):
    """Load config from URL"""
    if not server:
        raise HTTPException(status_code=500, detail="Server not initialized")

    url = request_data.get('url', '')
    if not url:
        return JSONResponse(content={
            "success": False,
            "message": "URL is required"
        })

    success = await server.load_config_from_url(url)

    return JSONResponse(content={
        "success": success,
        "message": "Config loaded from URL successfully" if success else "Failed to load config from URL"
    })


@app.post("/api/alerts/test", response_class=HTMLResponse)
async def api_test_alerts():
    """Test alert system"""
    if not server:
        raise HTTPException(status_code=500, detail="Server not initialized")

    result = await server.test_alerts()

    if result["success"]:
        return HTMLResponse(content=f'''
        <div class="bg-green-50 border border-green-200 rounded-lg p-4">
            <div class="flex items-center">
                <div class="flex-shrink-0">
                    <svg class="h-5 w-5 text-green-400" viewBox="0 0 20 20" fill="currentColor">
                        <path fill-rule="evenodd" d="M10 18a8 8 0 100-16 8 8 0 000 16zm3.707-9.293a1 1 0 00-1.414-1.414L9 10.586 7.707 9.293a1 1 0 00-1.414 1.414l2 2a1 1 0 001.414 0l4-4z" clip-rule="evenodd" />
                    </svg>
                </div>
                <div class="ml-3">
                    <p class="text-sm text-green-800">✅ {result["message"]}</p>
                </div>
            </div>
        </div>
        ''')
    else:
        return HTMLResponse(content=f'''
        <div class="bg-red-50 border border-red-200 rounded-lg p-4">
            <div class="flex items-center">
                <div class="flex-shrink-0">
                    <svg class="h-5 w-5 text-red-400" viewBox="0 0 20 20" fill="currentColor">
                        <path fill-rule="evenodd" d="M10 18a8 8 0 100-16 8 8 0 000 16zM8.707 7.293a1 1 0 00-1.414 1.414L8.586 10l-1.293 1.293a1 1 0 101.414 1.414L10 11.414l1.293 1.293a1 1 0 001.414-1.414L11.414 10l1.293-1.293a1 1 0 00-1.414-1.414L10 8.586 8.707 7.293z" clip-rule="evenodd" />
                    </svg>
                </div>
                <div class="ml-3">
                    <p class="text-sm text-red-800">❌ {result["message"]}</p>
                </div>
            </div>
        </div>
        ''')


@app.websocket("/ws/logs")
async def websocket_logs(websocket: WebSocket):
    """WebSocket endpoint for real-time log streaming"""
    await websocket.accept()

    if not server:
        await websocket.close(code=1000)
        return

    await server.add_log_connection(websocket)

    try:
        while True:
            # Keep connection alive and listen for client messages
            message = await websocket.receive_text()
            # Could handle log file selection here
    except WebSocketDisconnect:
        await server.remove_log_connection(websocket)


if __name__ == "__main__":
    import uvicorn

    # Get config file from command line or environment (optional now)
    config_file = None
    if len(sys.argv) > 1:
        config_file = sys.argv[1]
    else:
        config_file = os.environ.get('PHYTO_ARM_CONFIG')

    # Set environment variable for app startup (can be None)
    if config_file:
        os.environ['PHYTO_ARM_CONFIG'] = config_file
        logger.info(f"Starting PhytO-ARM Control Server 2.0 with config: {config_file}")
    else:
        # Remove the env var if it exists
        os.environ.pop('PHYTO_ARM_CONFIG', None)
        logger.info("Starting PhytO-ARM Control Server 2.0 without config - config must be loaded via web interface")

    uvicorn.run(
        app,
        host="0.0.0.0",
        port=8080,
        log_level="info"
    )
