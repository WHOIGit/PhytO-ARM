"""
PhytO-ARM Server - Main server class that manages all PhytO-ARM processes
"""

import asyncio
import glob
import json
import logging
import os
import re
import shlex
import subprocess
import tempfile
import urllib.error
import urllib.request
from datetime import datetime
from typing import Dict, List, Optional, Tuple

import yaml
from fastapi import WebSocket, WebSocketDisconnect

from .models import ConfigValidationResult, ProcessInfo, ProcessState
from .process import PhytoARMProcess

logger = logging.getLogger(__name__)


def _is_roscore_running() -> bool:
    """Check if roscore is running by trying to connect to the parameter server"""
    try:
        # Test if rosparam command is available
        result = subprocess.run(['which', 'rosparam'], capture_output=True, text=True, check=False)
        if result.returncode != 0:
            return False

        # Try to list parameters - this will fail if roscore isn't running
        cmd_result = subprocess.run(['rosparam', 'list'], capture_output=True, text=True, timeout=5, check=False)
        return cmd_result.returncode == 0
    except (subprocess.TimeoutExpired, subprocess.SubprocessError, FileNotFoundError, OSError):
        return False


def _get_ros_status() -> dict:
    """Get detailed ROS status information"""
    status = {
        "ros_tools_available": False,
        "roscore_running": False,
        "master_uri": None,
        "error": None
    }

    try:
        # Test if rosparam command is available
        result = subprocess.run(['which', 'rosparam'], capture_output=True, text=True, check=False)
        status["ros_tools_available"] = result.returncode == 0

        if not status["ros_tools_available"]:
            status["error"] = "ROS tools not available"
            return status

        status["master_uri"] = os.environ.get('ROS_MASTER_URI', 'http://localhost:11311')
        status["roscore_running"] = _is_roscore_running()
    except (subprocess.SubprocessError, FileNotFoundError, OSError) as e:
        status["error"] = str(e)

    return status


def _update_ros_parameters(config: dict) -> Tuple[bool, str]:
    """Update ROS parameters with the entire config using rosparam CLI"""
    ros_status = _get_ros_status()

    if not ros_status["ros_tools_available"]:
        return False, "ROS tools not available"

    if not ros_status["roscore_running"]:
        return False, "ROS core is not running"

    try:
        # Create temporary YAML file with all config parameters
        with tempfile.NamedTemporaryFile(mode='w', suffix='.yaml', delete=False) as tmp:
            yaml.dump(config, tmp)
            tmp_path = tmp.name

        try:
            # Use rosparam load to set all parameters at once
            cmd = ['rosparam', 'load', tmp_path]
            cmd_result = subprocess.run(cmd, capture_output=True, text=True, timeout=30, check=False)

            if cmd_result.returncode == 0:
                logger.info("Successfully updated ROS parameters via rosparam load")
                return True, "Parameters updated successfully"

            error_msg = cmd_result.stderr.strip() or cmd_result.stdout.strip() or "Unknown error"
            logger.error("rosparam load failed: %s", error_msg)
            return False, f"Failed to load parameters: {error_msg}"

        finally:
            # Clean up temporary file
            os.unlink(tmp_path)

    except subprocess.TimeoutExpired:
        return False, "rosparam load timed out"
    except (subprocess.SubprocessError, FileNotFoundError, OSError) as e:
        logger.error("Error updating ROS parameters: %s", e)
        return False, f"Error updating parameters: {str(e)}"


async def _wait_for_roscore_and_apply_config(server_instance, max_wait_seconds: int = 30):
    """Wait for roscore to be ready and then apply current config"""
    ros_status = _get_ros_status()
    if not ros_status["ros_tools_available"]:
        logger.warning("ROS tools not available - skipping config application")
        return

    logger.info("Waiting for roscore to be ready...")

    for _ in range(max_wait_seconds):
        if _is_roscore_running():
            logger.info("Roscore is ready, applying configuration...")

            if server_instance.config:
                success, message = _update_ros_parameters(server_instance.config)
                if success:
                    logger.info("Configuration applied to ROS parameter server after roscore startup")
                else:
                    logger.warning("Failed to apply config to ROS parameter server: %s", message)
            else:
                logger.warning("No config loaded to apply to ROS parameter server")
            return

        await asyncio.sleep(1)

    logger.warning("Roscore did not become ready within %d seconds", max_wait_seconds)


class PhytoARMServer:
    """Main server class that manages all PhytO-ARM processes"""

    def __init__(self, config_file_path: Optional[str] = None, config_schema: str = None):
        self.config_file = config_file_path
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

        # Temporary launch configs for cleanup
        self._temp_launch_configs: set = set()

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
                logger.info("Loading config file %s", self.config_file)
                await self.load_config_from_file(self.config_file)
            else:
                if self.config_file:
                    logger.warning("Config file %s not found, starting without config", self.config_file)
                else:
                    logger.info("Starting server without config - config must be loaded via web interface")

            # Start monitoring
            self._monitor_task = asyncio.create_task(self._monitor_processes())

            logger.info("PhytO-ARM server initialized successfully")

        except (OSError, yaml.YAMLError, ValueError) as e:
            logger.error("Failed to initialize server: %s", e)
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

            logger.info("Config loaded successfully from %s", source)

            # Optionally auto-start roscore when config is loaded
            auto_start_roscore = self.config.get('launch_args', {}).get('auto_start_roscore', False)
            if auto_start_roscore and not self._is_process_running("roscore"):
                logger.info("Auto-starting roscore because auto_start_roscore is enabled")
                await self.start_process("roscore")

        except (OSError, yaml.YAMLError, ValueError) as e:
            logger.error("Failed to apply config from %s: %s", source, e)
            raise

    async def load_config_from_file(self, config_path: str):
        """Load configuration from a file"""
        try:
            # Validate config
            from scripts.config_validation import validate_config
            logger.info("Validating config file %s", config_path)
            if not validate_config(config_path, self.config_schema):
                raise ValueError("Config validation failed")

            # Load config content
            with open(config_path, 'r', encoding='utf-8') as f:
                config_content = f.read()

            # Apply the config
            await self._apply_loaded_config(config_content, config_path, config_path)

        except (OSError, yaml.YAMLError, ValueError) as e:
            logger.error("Failed to load config from %s: %s", config_path, e)
            raise

    async def load_config_from_url(self, url: str) -> bool:
        """Load configuration from a URL"""
        try:
            logger.info("Downloading config from %s", url)

            # Download config
            with urllib.request.urlopen(url) as response:
                config_content = response.read().decode('utf-8')

            # Validate config
            validation = self.validate_config_content(config_content)
            if not validation.valid:
                logger.error("Config from URL %s failed validation: %s", url, validation.errors)
                return False

            # Apply the config
            await self._apply_loaded_config(config_content, f"URL {url}")
            return True

        except (OSError, urllib.error.URLError, yaml.YAMLError, ValueError) as e:
            logger.error("Failed to load config from URL %s: %s", url, e)
            return False

    def _discover_launch_files(self):
        """Discover available launch files in the phyto_arm package"""
        parent_dir = os.path.dirname(os.path.dirname(__file__))
        launch_dir = os.path.join(parent_dir, 'src', 'phyto_arm', 'launch')

        if not os.path.exists(launch_dir):
            logger.warning("Launch directory not found: %s", launch_dir)
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
                logger.info("Discovered launch file: %s -> %s", name, filename)

        logger.info("Found %d launch configurations", len(self.launch_configs))

    def _extract_launch_description(self, launch_file_path: str) -> str:
        """Extract description from launch file comments or XML"""
        try:
            with open(launch_file_path, 'r', encoding='utf-8') as f:
                content = f.read()

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

        except (OSError, IOError) as e:
            logger.warning("Failed to extract description from %s: %s", launch_file_path, e)
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
        if process_name == "rosbag":
            return {
                "description": "Data Logging - Records ROS topics to bag files",
                "type": "system",
                "category": "logging"
            }
        # Launch file processes
        if process_name in self.launch_configs:
            config = self.launch_configs[process_name]
            if isinstance(config, dict):
                return {
                    "description": config.get('description', f"Launch configuration: {process_name}"),
                    "type": "launch",
                    "category": self._categorize_process(process_name),
                    "filename": config.get('filename', ''),
                    "path": config.get('path', '')
                }
            # Handle legacy string format
            return {
                "description": f"Launch configuration: {process_name}",
                "type": "launch",
                "category": self._categorize_process(process_name),
                "filename": config
            }

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
        if any(word in name_lower for word in ['mock', 'sim', 'test']):
            return "simulation"
        if any(word in name_lower for word in ['sensor', 'ctd', 'gps', 'camera']):
            return "sensors"
        if any(word in name_lower for word in ['main', 'core', 'base']):
            return "core"

        return "other"

    def _prep_environment(self) -> Dict[str, str]:
        """Prepare ROS environment - adapted from original phyto-arm script"""
        parent_dir = os.path.dirname(os.path.dirname(__file__))
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

        try:
            # Create temporary file with current config
            fd, temp_path = tempfile.mkstemp(suffix='.yaml', prefix='phyto_arm_launch_')
            with os.fdopen(fd, 'w') as f:
                # Use preserved formatting if available, otherwise dump current config
                if self.config_content:
                    f.write(self.config_content)
                else:
                    yaml.dump(self.config, f)

            logger.debug("Created temporary config file for launch: %s", temp_path)
            return temp_path

        except (OSError, IOError) as e:
            logger.error("Failed to create temporary config file: %s", e)
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
            logger.error("Unknown process: %s", process_name)
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
            logger.warning("Process %s not found", process_name)
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

        except (OSError, IOError) as e:
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
        except (OSError, IOError) as e:
            return {"error": f"Error reading file {filename}: {str(e)}", "lines": []}

    def get_config_content(self) -> str:
        """Get current config file content with original formatting preserved"""
        try:
            # Return the preserved content if available
            if self.config_content:
                return self.config_content

            # Fallback: read from file if no content stored
            if self.config_file and os.path.exists(self.config_file):
                with open(self.config_file, 'r', encoding='utf-8') as f:
                    return f.read()

            return ""
        except (OSError, IOError) as e:
            logger.error("Failed to get config content: %s", e)
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
            ros_status = _get_ros_status()
            if not ros_status["ros_tools_available"]:
                warnings.append("ROS tools not available - parameters cannot be updated")
            elif not ros_status["roscore_running"]:
                warnings.append("ROS core is not running - parameters will be applied when roscore starts")

            return ConfigValidationResult(valid=True, warnings=warnings)

        except yaml.YAMLError as e:
            return ConfigValidationResult(
                valid=False,
                errors=[f"YAML syntax error: {str(e)}"]
            )
        except (TypeError, AttributeError) as e:
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

            logger.warning("Config applied to memory but ROS update failed: %s", ros_message)
            return True, f"Configuration applied to memory. ROS parameters: {ros_message}"

        except (yaml.YAMLError, TypeError, AttributeError, OSError) as e:
            logger.error("Failed to apply config changes: %s", e)
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
        except (yaml.YAMLError, TypeError, AttributeError) as e:
            logger.error("Failed to reset config: %s", e)
            return ""

    async def _monitor_processes(self):
        """Background task to monitor process health"""
        while not self._shutdown:
            try:
                for name, process in list(self.processes.items()):
                    # Check if process failed
                    if not process.is_running() and process.info.state == ProcessState.FAILED:
                        logger.error("Process %s has failed", name)

                        # Send alerts if configured
                        await self._send_alerts(name)

                await asyncio.sleep(5)  # Check every 5 seconds

            except (OSError, asyncio.CancelledError) as e:
                logger.error("Error in process monitor: %s", e)
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
                    deploy_str = f"Deployment: _{deployment}_"
                    if test_mode:
                        time_str = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
                        message = {
                            'text': f'🧪 *PhytO-ARM Alert Test*\n - {deploy_str}\n - Test Time: {time_str}'
                        }
                    else:
                        message = {
                            'text': f'*PhytO-ARM process failed*\n - {deploy_str}\n - Process: _{process_name}_'
                        }

                    urllib.request.urlopen(
                        alert['url'],
                        json.dumps(message).encode()
                    )

                    if test_mode:
                        logger.info("Test alert sent successfully")
                    else:
                        logger.info("Alert sent for %s", process_name)
                except (urllib.error.URLError, OSError, ValueError) as e:
                    logger.error("Failed to send alert: %s", e)

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
        except (urllib.error.URLError, OSError, ValueError) as e:
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
                        except (OSError, IOError) as e:
                            logger.warning("Error reading log file %s: %s", filename, e)

                await asyncio.sleep(1)  # Check every second

        except asyncio.CancelledError:
            pass
        except (OSError, IOError) as e:
            logger.error("Error monitoring log file %s: %s", filename, e)

    async def broadcast_log_update(self, log_data: dict):
        """Broadcast log updates to all connected WebSocket clients"""
        if self.log_connections:
            message = json.dumps(log_data)
            disconnected = []
            for websocket in self.log_connections:
                try:
                    await websocket.send_text(message)
                except (WebSocketDisconnect, ConnectionResetError, OSError):
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
        for _, task in self.log_file_monitors.items():
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
                        logger.debug("Cleaned up temporary config file: %s", temp_file)
                except (OSError, IOError) as e:
                    logger.warning("Failed to clean up temporary config file %s: %s", temp_file, e)

        logger.info("PhytO-ARM server shutdown complete")
