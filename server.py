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
import urllib.request
from contextlib import asynccontextmanager
from datetime import datetime
from enum import Enum
from typing import Dict, Optional, List, Tuple
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

# Check if ROS tools are available
try:
    # Test if rosparam command is available
    result = subprocess.run(['which', 'rosparam'], capture_output=True, text=True)
    ROS_TOOLS_AVAILABLE = result.returncode == 0
    if ROS_TOOLS_AVAILABLE:
        logger.info("ROS tools (rosparam) available")
    else:
        logger.warning("ROS tools not found in PATH")
except Exception as e:
    ROS_TOOLS_AVAILABLE = False
    logger.warning(f"Failed to check for ROS tools: {e}")


def _is_roscore_running() -> bool:
    """Check if roscore is running by trying to connect to the parameter server"""
    if not ROS_TOOLS_AVAILABLE:
        return False

    try:
        # Try to list parameters - this will fail if roscore isn't running
        result = subprocess.run(['rosparam', 'list'], capture_output=True, text=True, timeout=5)
        return result.returncode == 0
    except subprocess.TimeoutExpired:
        logger.debug("rosparam list timed out - roscore likely not running")
        return False
    except Exception as e:
        logger.debug(f"ROS parameter server not available: {e}")
        return False


def _get_ros_status() -> dict:
    """Get detailed ROS status information"""
    status = {
        "ros_tools_available": ROS_TOOLS_AVAILABLE,
        "roscore_running": False,
        "master_uri": None,
        "error": None
    }

    if not ROS_TOOLS_AVAILABLE:
        status["error"] = "ROS tools not available"
        return status

    try:
        status["master_uri"] = os.environ.get('ROS_MASTER_URI', 'http://localhost:11311')
        status["roscore_running"] = _is_roscore_running()
    except Exception as e:
        status["error"] = str(e)

    return status


def _update_ros_parameters(config: dict) -> Tuple[bool, str]:
    """Update ROS parameters with the entire config using rosparam CLI"""
    if not ROS_TOOLS_AVAILABLE:
        return False, "ROS tools not available"

    if not _is_roscore_running():
        return False, "ROS core is not running"

    import tempfile
    try:
        # Create temporary YAML file with all config parameters
        with tempfile.NamedTemporaryFile(mode='w', suffix='.yaml', delete=False) as tmp:
            yaml.dump(config, tmp)
            tmp_path = tmp.name

        try:
            # Use rosparam load to set all parameters at once
            cmd = ['rosparam', 'load', tmp_path]
            result = subprocess.run(cmd, capture_output=True, text=True, timeout=30)

            if result.returncode == 0:
                logger.info("Successfully updated ROS parameters via rosparam load")
                return True, "Parameters updated successfully"
            else:
                error_msg = result.stderr.strip() or result.stdout.strip() or "Unknown error"
                logger.error(f"rosparam load failed: {error_msg}")
                return False, f"Failed to load parameters: {error_msg}"

        finally:
            # Clean up temporary file
            os.unlink(tmp_path)

    except subprocess.TimeoutExpired:
        return False, "rosparam load timed out"
    except Exception as e:
        logger.error(f"Error updating ROS parameters: {e}")
        return False, f"Error updating parameters: {str(e)}"


async def _wait_for_roscore_and_apply_config(server_instance, max_wait_seconds: int = 30):
    """Wait for roscore to be ready and then apply current config"""
    if not ROS_TOOLS_AVAILABLE:
        logger.warning("ROS tools not available - skipping config application")
        return

    logger.info("Waiting for roscore to be ready...")

    for attempt in range(max_wait_seconds):
        if _is_roscore_running():
            logger.info("Roscore is ready, applying configuration...")

            if server_instance.config:
                success, message = _update_ros_parameters(server_instance.config)
                if success:
                    logger.info("Configuration applied to ROS parameter server after roscore startup")
                else:
                    logger.warning(f"Failed to apply config to ROS parameter server: {message}")
            else:
                logger.warning("No config loaded to apply to ROS parameter server")
            return

        await asyncio.sleep(1)

    logger.warning(f"Roscore did not become ready within {max_wait_seconds} seconds")


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
        self.config_content = None  # Store original YAML text with formatting
        self.original_config_content = None  # Store original YAML text for reset
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
        self.log_file_monitors: Dict[str, asyncio.Task] = {}
        self.log_file_positions: Dict[str, int] = {}

    def _is_process_running(self, process_name: str) -> bool:
        """Check if a process is currently running"""
        if process_name in self.processes:
            return self.processes[process_name].is_running()
        return False

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

    async def _apply_loaded_config(self, config_content: str, source: str, config_path: str = None):
        """Apply loaded config content - common logic for file and URL loading"""
        try:
            # Load the validated config
            self.config = yaml.safe_load(config_content)
            self.config_content = config_content
            self.original_config = yaml.safe_load(config_content)  # Parse fresh copy
            self.original_config_content = config_content
            self.config_loaded = True

            # Set config file path if provided
            if config_path:
                self.config_file = config_path

            # Setup environment
            self.env = self._prep_environment()

            # Discover available launch files
            self._discover_launch_files()

            logger.info(f"Config loaded successfully from {source}")

            # Optionally auto-start roscore when config is loaded
            auto_start_roscore = self.config.get('launch_args', {}).get('auto_start_roscore', False)
            if auto_start_roscore and not self._is_process_running("roscore"):
                logger.info("Auto-starting roscore because auto_start_roscore is enabled")
                await self.start_process("roscore")

        except Exception as e:
            logger.error(f"Failed to apply config from {source}: {e}")
            raise

    async def load_config_from_file(self, config_path: str):
        """Load configuration from a file"""
        try:
            # Validate config
            logger.info(f"Validating config file {config_path}")
            if not validate_config(config_path, self.config_schema):
                raise ValueError("Config validation failed")

            # Load config content
            with open(config_path, 'r') as f:
                config_content = f.read()

            # Apply the config
            await self._apply_loaded_config(config_content, config_path, config_path)

        except Exception as e:
            logger.error(f"Failed to load config from {config_path}: {e}")
            raise

    async def load_config_from_url(self, url: str) -> bool:
        """Load configuration from a URL"""
        try:
            logger.info(f"Downloading config from {url}")

            # Download config
            with urllib.request.urlopen(url) as response:
                config_content = response.read().decode('utf-8')

            # Validate config
            validation = self.validate_config_content(config_content)
            if not validation.valid:
                logger.error(f"Config from URL {url} failed validation: {validation.errors}")
                return False

            # Apply the config
            await self._apply_loaded_config(config_content, f"URL {url}")
            return True

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


    def _create_temp_config_file(self) -> Optional[str]:
        """Create temporary config file from in-memory config for roslaunch"""
        if not self.config:
            return None

        import tempfile
        try:
            # Create temporary file with current config
            fd, temp_path = tempfile.mkstemp(suffix='.yaml', prefix='phyto_arm_launch_')
            with os.fdopen(fd, 'w') as f:
                # Use preserved formatting if available, otherwise dump current config
                if self.config_content:
                    f.write(self.config_content)
                else:
                    yaml.dump(self.config, f)

            logger.debug(f"Created temporary config file for launch: {temp_path}")
            return temp_path

        except Exception as e:
            logger.error(f"Failed to create temporary config file: {e}")
            return None

    def _build_roslaunch_command(self, package: str, launchfile: str) -> str:
        """Build roslaunch command with config args"""
        rl_args = [
            'roslaunch',
            '--wait',
            '--required',
            '--skip-log-check',
            package, launchfile
        ]

        # Always create temp file with current in-memory config for consistency
        if self.config:
            temp_config_path = self._create_temp_config_file()
            if temp_config_path:
                rl_args.append(f'config_file:={os.path.abspath(temp_config_path)}')
                # Store path for cleanup later
                if not hasattr(self, '_temp_launch_configs'):
                    self._temp_launch_configs = set()
                self._temp_launch_configs.add(temp_config_path)

        # Pass launch args directly (these override file params)
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
        success = process.start()

        # If roscore was started successfully, apply config after it's ready
        if success and process_name == "roscore":
            # Start background task to wait for roscore and apply config
            asyncio.create_task(_wait_for_roscore_and_apply_config(self))

        return success

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
        """Get current config file content with original formatting preserved"""
        try:
            # Return the preserved content if available
            if self.config_content:
                return self.config_content

            # Fallback: read from file if no content stored
            if self.config_file and os.path.exists(self.config_file):
                with open(self.config_file, 'r') as f:
                    return f.read()
            else:
                return ""
        except Exception as e:
            logger.error(f"Failed to get config content: {e}")
            return ""

    def has_config(self) -> bool:
        """Check if a valid config is loaded"""
        return self.config_loaded and self.config is not None

    def validate_config_content(self, content: str) -> ConfigValidationResult:
        """Validate config content"""
        try:
            # Parse YAML to check syntax
            config_data = yaml.safe_load(content)

            # Basic validation - check if it's a dictionary
            if not isinstance(config_data, dict):
                return ConfigValidationResult(
                    valid=False,
                    errors=["Config must be a YAML dictionary"]
                )

            # Basic validation passed, but add warnings about ROS connectivity
            warnings = []
            if not ROS_TOOLS_AVAILABLE:
                warnings.append("ROS tools not available - parameters cannot be updated")
            elif not _is_roscore_running():
                warnings.append("ROS core is not running - parameters will be applied when roscore starts")

            return ConfigValidationResult(valid=True, warnings=warnings)

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

    async def apply_config_changes(self, content: str) -> Tuple[bool, str]:
        """Apply config changes to memory and update ROS parameters (no file changes)"""
        try:
            # Validate first
            validation = self.validate_config_content(content)
            if not validation.valid:
                return False, f"Validation failed: {', '.join(validation.errors)}"

            # Parse new config
            new_config = yaml.safe_load(content)

            # Update internal config (in-memory only)
            self.config = new_config
            self.config_content = content  # Preserve formatting
            self.config_loaded = True  # Mark config as properly loaded

            # Set original config for reset functionality if not already set
            if not self.original_config:
                self.original_config = yaml.safe_load(content)  # Parse fresh copy
                self.original_config_content = content

            # Update environment if needed
            self.env = self._prep_environment()

            # Discover available launch files
            self._discover_launch_files()

            # Update ROS parameters if roscore is running
            ros_success, ros_message = _update_ros_parameters(new_config)

            if ros_success:
                logger.info("Config changes applied to memory and ROS parameter server successfully")
                return True, "Configuration applied successfully to memory and ROS parameters"
            else:
                logger.warning(f"Config applied to memory but ROS update failed: {ros_message}")
                return True, f"Configuration applied to memory. ROS parameters: {ros_message}"

        except Exception as e:
            logger.error(f"Failed to apply config changes: {e}")
            return False, f"Failed to apply configuration: {str(e)}"

    def reset_config_to_original(self) -> str:
        """Reset config to original state (in memory only)"""
        try:
            # Reset in-memory config to original
            self.config = yaml.safe_load(self.original_config_content)  # Parse original content
            self.config_content = self.original_config_content  # Restore original formatting

            # Update environment if needed
            self.env = self._prep_environment()

            # Return original content with preserved formatting
            return self.original_config_content
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

    async def add_log_connection(self, websocket: WebSocket, log_filename: str = None):
        """Add a WebSocket connection for log streaming"""
        self.log_connections.append(websocket)

        # Start monitoring the specific log file if provided
        if log_filename:
            await self.start_log_file_monitor(log_filename)

    async def remove_log_connection(self, websocket: WebSocket):
        """Remove a WebSocket connection"""
        if websocket in self.log_connections:
            self.log_connections.remove(websocket)

    async def start_log_file_monitor(self, filename: str):
        """Start monitoring a specific log file for changes"""
        if filename in self.log_file_monitors:
            return  # Already monitoring

        log_dir = self.config.get('launch_args', {}).get('log_dir') if self.config else None
        if not log_dir:
            log_dir = os.environ.get('ROS_LOG_DIR', '/tmp')

        file_path = os.path.join(log_dir, 'latest', filename)

        if os.path.exists(file_path):
            # Start from end of file
            self.log_file_positions[filename] = os.path.getsize(file_path)

            # Create monitoring task
            task = asyncio.create_task(self._monitor_log_file(filename, file_path))
            self.log_file_monitors[filename] = task

    async def _monitor_log_file(self, filename: str, file_path: str):
        """Monitor a log file for new content and broadcast updates"""
        try:
            while not self._shutdown:
                if os.path.exists(file_path):
                    current_size = os.path.getsize(file_path)
                    last_position = self.log_file_positions.get(filename, 0)

                    if current_size > last_position:
                        # Read new content
                        try:
                            with open(file_path, 'r', encoding='utf-8', errors='ignore') as f:
                                f.seek(last_position)
                                new_content = f.read()

                                if new_content.strip():
                                    # Broadcast new content to WebSocket clients
                                    await self.broadcast_log_update({
                                        'type': 'log_update',
                                        'filename': filename,
                                        'content': new_content
                                    })

                                self.log_file_positions[filename] = current_size
                        except Exception as e:
                            logger.warning(f"Error reading log file {filename}: {e}")

                await asyncio.sleep(1)  # Check every second

        except asyncio.CancelledError:
            pass
        except Exception as e:
            logger.error(f"Error monitoring log file {filename}: {e}")

    async def broadcast_log_update(self, log_data: dict):
        """Broadcast log updates to all connected WebSocket clients"""
        if self.log_connections:
            message = json.dumps(log_data)
            disconnected = []
            for websocket in self.log_connections:
                try:
                    await websocket.send_text(message)
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

        # Cancel all log file monitoring tasks
        for filename, task in self.log_file_monitors.items():
            task.cancel()
        self.log_file_monitors.clear()

        # Stop all processes in reverse order (roscore last)
        stop_order = ["rosbag"] + [name for name in self.processes if name not in ["roscore", "rosbag"]] + ["roscore"]

        for name in stop_order:
            if name in self.processes:
                await self.stop_process(name)

        # Clean up temporary config files
        if hasattr(self, '_temp_launch_configs'):
            for temp_file in self._temp_launch_configs:
                try:
                    if os.path.exists(temp_file):
                        os.unlink(temp_file)
                        logger.debug(f"Cleaned up temporary config file: {temp_file}")
                except Exception as e:
                    logger.warning(f"Failed to clean up temporary config file {temp_file}: {e}")

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
app = FastAPI(title="PhytO-ARM Control Interface", version="2.0.0", lifespan=lifespan)


@app.get("/", response_class=HTMLResponse)
async def get_dashboard():
    """Main dashboard page with Tailwind CSS and HTMX"""
    try:
        with open("server/templates/dashboard.html", "r") as f:
            content = f.read()
        return HTMLResponse(content=content)
    except FileNotFoundError:
        return HTMLResponse(content="<h1>Dashboard template not found</h1>", status_code=500)


# Static file serving for JS libraries
@app.get("/static/tailwind-3.4.17.js")
async def get_tailwind():
    with open("server/tailwind-3.4.17.js", "r") as f:
        return HTMLResponse(content=f.read(), media_type="application/javascript")


@app.get("/static/htmx-2.0.7.min.js")
async def get_htmx():
    with open("server/htmx-2.0.7.min.js", "r") as f:
        return HTMLResponse(content=f.read(), media_type="application/javascript")


@app.get("/static/htmx-ext-ws-2.0.2.js")
async def get_htmx_ws():
    with open("server/htmx-ext-ws-2.0.2.js", "r") as f:
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
        "config_loaded": server.has_config(),
        "ros_status": _get_ros_status()
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
        html_parts.append(_load_template("config_warning.html"))

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


def _load_template(template_name: str) -> str:
    """Load HTML template from file"""
    try:
        with open(f"server/templates/{template_name}", "r") as f:
            return f.read()
    except FileNotFoundError:
        logger.error(f"Template {template_name} not found")
        return f"<div>Template {template_name} not found</div>"


def _render_process_card(process_info: ProcessInfo, metadata: dict, config_loaded: bool = True) -> str:
    """Render a single process card using template"""
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

    # Load template and substitute values
    template = _load_template("process_card.html")
    return template.format(
        category_class=category_class,
        process_name=process_info.name,
        description=metadata.get('description', 'No description'),
        type_info=type_info,
        config_warning=config_warning,
        state_class=state_class,
        state_upper=state.upper(),
        category_title=metadata.get('category', 'other').title(),
        pid_info=pid_info,
        started_at=started_at,
        restart_info=restart_info,
        start_disabled_attr="disabled" if start_disabled else "",
        stop_disabled_attr="disabled" if stop_disabled else ""
    )


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
    success, message = await server.apply_config_changes(content)

    return JSONResponse(content={
        "success": success,
        "message": message,
        "errors": [] if success else [message],
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
            # Listen for client messages (log file selection)
            message = await websocket.receive_text()
            try:
                data = json.loads(message)
                if data.get('type') == 'select_log_file':
                    filename = data.get('filename')
                    if filename:
                        # Start monitoring this log file
                        await server.start_log_file_monitor(filename)

                        # Send initial content
                        log_data = server.get_log_file_content(filename, max_lines=200)
                        if not log_data.get('error'):
                            await websocket.send_text(json.dumps({
                                'type': 'initial_content',
                                'filename': filename,
                                'content': '\n'.join(log_data['lines'])
                            }))
            except json.JSONDecodeError:
                logger.warning(f"Invalid JSON message from WebSocket: {message}")
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
        logger.info(f"Starting PhytO-ARM Control Server with config: {config_file}")
    else:
        # Remove the env var if it exists
        os.environ.pop('PHYTO_ARM_CONFIG', None)
        logger.info("Starting PhytO-ARM Control Server without config - config must be loaded via web interface")

    uvicorn.run(
        app,
        host="0.0.0.0",
        port=8080,
        log_level="info"
    )
