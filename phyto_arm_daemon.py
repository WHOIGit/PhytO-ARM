#!/usr/bin/env python3
"""
PhytO-ARM Daemon - Always-on server for managing PhytO-ARM ROS processes

This replaces the original phyto-arm script with a persistent daemon that:
- Manages ROS process lifecycle (roscore, rosbag, arms)  
- Provides web interface for status and control
- Preserves config validation and alert functionality
- Eliminates need for screen/tmux wrappers
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
from typing import Dict, Optional, List
import glob

import yaml
from fastapi import FastAPI, HTTPException
from fastapi.responses import HTMLResponse
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

    def _read_output(self):
        """Legacy method - now unused since we read ROS log files directly"""
        pass


class PhytoARMDaemon:
    """Main daemon class that manages all PhytO-ARM processes"""

    def __init__(self, config_file: str, config_schema: str = None):
        self.config_file = config_file
        self.config_schema = config_schema or "./configs/example.yaml"
        self.config = None
        self.env = None

        # Process registry
        self.processes: Dict[str, PhytoARMProcess] = {}

        # Available launch configurations (discovered at runtime)
        self.launch_configs = {}

        # Background monitoring
        self._monitor_task = None
        self._shutdown = False

    async def initialize(self):
        """Initialize the daemon - load config, setup environment"""
        try:
            # Validate config
            logger.info(f"Validating config file {self.config_file}")
            if not validate_config(self.config_file, self.config_schema):
                raise ValueError("Config validation failed")

            # Load config
            with open(self.config_file, 'r') as f:
                self.config = yaml.safe_load(f)

            # Setup environment
            self.env = self._prep_environment()

            # Discover available launch files
            self._discover_launch_files()

            # Start monitoring
            self._monitor_task = asyncio.create_task(self._monitor_processes())

            logger.info("PhytO-ARM daemon initialized successfully")

        except Exception as e:
            logger.error(f"Failed to initialize daemon: {e}")
            raise

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
                self.launch_configs[name] = filename
                logger.info(f"Discovered launch file: {name} -> {filename}")

        logger.info(f"Found {len(self.launch_configs)} launch configurations")

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
            launchfile = self.launch_configs[process_name]
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

    async def _send_alerts(self, process_name: str):
        """Send alerts when process fails - adapted from original"""
        alerts = self.config.get('alerts', [])
        deployment = self.config.get('name', 'unknown')

        for alert in alerts:
            if alert.get('type') == 'slack' and alert.get('url'):
                try:
                    message = {
                        'text': f'*PhytO-ARM process failed*\n - Deployment: _{deployment}_\n - Process: _{process_name}_'
                    }
                    urllib.request.urlopen(
                        alert['url'],
                        json.dumps(message).encode()
                    )
                    logger.info(f"Alert sent for {process_name}")
                except Exception as e:
                    logger.error(f"Failed to send alert: {e}")

    async def shutdown(self):
        """Gracefully shutdown all processes"""
        logger.info("Shutting down PhytO-ARM daemon")
        self._shutdown = True

        if self._monitor_task:
            self._monitor_task.cancel()

        # Stop all processes in reverse order (roscore last)
        stop_order = ["rosbag"] + [name for name in self.processes if name not in ["roscore", "rosbag"]] + ["roscore"]

        for name in stop_order:
            if name in self.processes:
                await self.stop_process(name)

        logger.info("PhytO-ARM daemon shutdown complete")


# Global daemon instance
daemon: Optional[PhytoARMDaemon] = None


@asynccontextmanager
async def lifespan(app: FastAPI):
    # Startup
    global daemon
    config_file = os.environ.get('PHYTO_ARM_CONFIG', '/configs/config.yaml')
    daemon = PhytoARMDaemon(config_file)
    await daemon.initialize()

    yield

    # Shutdown
    if daemon:
        await daemon.shutdown()


# FastAPI app
app = FastAPI(title="PhytO-ARM Control Interface", version="1.0.0", lifespan=lifespan)


@app.get("/", response_class=HTMLResponse)
async def get_dashboard():
    """Main dashboard page"""
    return """
    <!DOCTYPE html>
    <html>
    <head>
        <title>PhytO-ARM Control Dashboard</title>
        <meta charset="UTF-8">
        <meta name="viewport" content="width=device-width, initial-scale=1.0">
        <style>
            body { font-family: Arial, sans-serif; margin: 20px; background: #f5f5f5; }
            .container { max-width: 1200px; margin: 0 auto; }
            .header { background: #2c3e50; color: white; padding: 20px; border-radius: 8px; margin-bottom: 20px; }
            .process-grid { display: grid; grid-template-columns: repeat(auto-fit, minmax(300px, 1fr)); gap: 20px; }
            .process-card { background: white; border-radius: 8px; padding: 20px; box-shadow: 0 2px 4px rgba(0,0,0,0.1); }
            .status { padding: 4px 8px; border-radius: 4px; color: white; font-weight: bold; }
            .status.running { background: #27ae60; }
            .status.stopped { background: #95a5a6; }
            .status.starting { background: #f39c12; }
            .status.stopping { background: #e67e22; }
            .status.failed { background: #e74c3c; }
            .controls { margin-top: 15px; }
            .btn { padding: 8px 16px; margin: 4px; border: none; border-radius: 4px; cursor: pointer; font-weight: bold; }
            .btn.start { background: #27ae60; color: white; }
            .btn.stop { background: #e74c3c; color: white; }
            .btn:disabled { background: #bdc3c7; cursor: not-allowed; }
            .refresh-btn { position: fixed; top: 20px; right: 20px; z-index: 1000; }
            .logs { margin-top: 20px; background: #2c3e50; color: #ecf0f1; padding: 15px; border-radius: 4px; max-height: 300px; overflow-y: auto; }
            .btn.logs { background: #3498db; color: white; }
            .modal { display: none; position: fixed; z-index: 2000; left: 0; top: 0; width: 100%; height: 100%; background-color: rgba(0,0,0,0.5); }
            .modal-content { background-color: #fefefe; margin: 5% auto; padding: 20px; border-radius: 8px; width: 80%; max-width: 800px; max-height: 80%; overflow-y: auto; }
            .close { color: #aaa; float: right; font-size: 28px; font-weight: bold; cursor: pointer; }
            .close:hover { color: black; }
            .log-output { background: #2c3e50; color: #ecf0f1; padding: 15px; border-radius: 4px; max-height: 400px; overflow-y: auto; font-family: monospace; white-space: pre-wrap; }
            .tabs { display: flex; border-bottom: 1px solid #ddd; margin-bottom: 20px; }
            .tab { padding: 12px 24px; cursor: pointer; border: none; background: #f1f1f1; margin-right: 4px; border-radius: 4px 4px 0 0; }
            .tab.active { background: #2c3e50; color: white; }
            .tab-content { display: none; }
            .tab-content.active { display: block; }
            .log-file-list { display: grid; grid-template-columns: repeat(auto-fit, minmax(300px, 1fr)); gap: 15px; margin-bottom: 20px; }
            .log-file-card { background: white; padding: 15px; border-radius: 8px; box-shadow: 0 2px 4px rgba(0,0,0,0.1); cursor: pointer; border: 2px solid transparent; }
            .log-file-card:hover { border-color: #3498db; }
            .log-file-card.selected { border-color: #2c3e50; background: #ecf0f1; }
            .log-viewer { background: #2c3e50; color: #ecf0f1; padding: 20px; border-radius: 8px; height: 500px; overflow-y: auto; font-family: monospace; white-space: pre-wrap; }
        </style>
    </head>
    <body>
        <button class="btn start refresh-btn" onclick="location.reload()">🔄 Refresh</button>

        <div class="container">
            <div class="header">
                <h1>🦠 PhytO-ARM Control Dashboard</h1>
            </div>

            <!-- Tabs -->
            <div class="tabs">
                <button class="tab active" onclick="switchTab('processes')">🔧 Processes</button>
                <button class="tab" onclick="switchTab('logs')">📄 Logs</button>
            </div>

            <!-- Processes Tab -->
            <div id="processes-tab" class="tab-content active">
                <div id="processes" class="process-grid">
                    <!-- Processes will be loaded here -->
                </div>
            </div>

            <!-- Logs Tab -->
            <div id="logs-tab" class="tab-content">
                <div class="log-file-list" id="logFileList">
                    <!-- Log files will be loaded here -->
                </div>
                <div id="logViewer" class="log-viewer">
                    Select a log file above to view its contents
                </div>
                <div style="margin-top: 10px;">
                    <button class="btn start" onclick="refreshLogContent()">🔄 Refresh Log</button>
                    <button class="btn start" onclick="loadLogFiles()">📁 Refresh File List</button>
                </div>
            </div>
        </div>

        <script>
            let launchConfigs = {};

            async function loadLaunchConfigs() {
                try {
                    const response = await fetch('/api/launch_configs');
                    launchConfigs = await response.json();
                } catch (error) {
                    console.error('Failed to load launch configs:', error);
                }
            }

            async function loadStatus() {
                try {
                    const response = await fetch('/api/status');
                    const processes = await response.json();

                    const container = document.getElementById('processes');
                    container.innerHTML = '';

                    // Define process order and descriptions
                    const processOrder = ['roscore', 'rosbag'];
                    const processDescriptions = {
                        'roscore': 'ROS Master - Core communication hub',
                        'rosbag': 'Data Logging - Records all ROS topics',  
                        'main': 'Main Processes - Core PhytO-ARM systems',
                        'arm_ifcb': 'IFCB Arm - Imaging FlowCytobot missions',
                        'arm_chanos': 'Chanos Arm - Multi-sensor payload',
                        'arm_oleander': 'Oleander Arm - Custom payload',
                        'mock_arm_ifcb': 'Mock IFCB Arm - Simulated IFCB missions',
                        'mock_arm_chanos': 'Mock Chanos Arm - Simulated Chanos missions'
                    };

                    // Add core processes first
                    processOrder.forEach(name => {
                        const process = processes[name] || {name: name, state: 'stopped'};
                        const card = createProcessCard(process, processDescriptions[name] || '');
                        container.appendChild(card);
                    });

                    // Add all discovered launch configs
                    Object.keys(launchConfigs).sort().forEach(name => {
                        const process = processes[name] || {name: name, state: 'stopped'};
                        const description = processDescriptions[name] || `Launch configuration: ${name}`;
                        const card = createProcessCard(process, description);
                        container.appendChild(card);
                    });

                } catch (error) {
                    console.error('Failed to load status:', error);
                }
            }

            function createProcessCard(process, description) {
                const card = document.createElement('div');
                card.className = 'process-card';

                const isRunning = process.state === 'running';
                const isTransitioning = process.state === 'starting' || process.state === 'stopping';

                card.innerHTML = `
                    <h3>${process.name}</h3>
                    <p>${description}</p>
                    <div class="status ${process.state}">${process.state.toUpperCase()}</div>
                    ${process.pid ? `<p><strong>PID:</strong> ${process.pid}</p>` : ''}
                    ${process.started_at ? `<p><strong>Started:</strong> ${new Date(process.started_at).toLocaleString()}</p>` : ''}
                    ${process.restart_count > 0 ? `<p><strong>Restarts:</strong> ${process.restart_count}</p>` : ''}

                    <div class="controls">
                        <button class="btn start" ${isRunning || isTransitioning ? 'disabled' : ''} 
                                onclick="startProcess('${process.name}')">▶️ Start</button>
                        <button class="btn stop" ${!isRunning || isTransitioning ? 'disabled' : ''} 
                                onclick="stopProcess('${process.name}')">⏹️ Stop</button>
                    </div>
                `;

                return card;
            }

            async function startProcess(name) {
                try {
                    const response = await fetch(`/api/start/${name}`, { method: 'POST' });
                    if (response.ok) {
                        setTimeout(loadStatus, 1000); // Refresh after 1 second
                    }
                } catch (error) {
                    console.error('Failed to start process:', error);
                }
            }

            async function stopProcess(name) {
                try {
                    const response = await fetch(`/api/stop/${name}`, { method: 'POST' });  
                    if (response.ok) {
                        setTimeout(loadStatus, 1000); // Refresh after 1 second
                    }
                } catch (error) {
                    console.error('Failed to stop process:', error);
                }
            }

            // Tab switching
            function switchTab(tabName) {
                // Hide all tab contents
                document.querySelectorAll('.tab-content').forEach(content => {
                    content.classList.remove('active');
                });

                // Remove active class from all tabs
                document.querySelectorAll('.tab').forEach(tab => {
                    tab.classList.remove('active');
                });

                // Show selected tab content
                document.getElementById(tabName + '-tab').classList.add('active');

                // Add active class to clicked tab
                event.target.classList.add('active');

                // Load log files when switching to logs tab
                if (tabName === 'logs') {
                    loadLogFiles();
                }
            }

            let currentLogFile = '';

            async function loadLogFiles() {
                try {
                    const response = await fetch('/api/log_files');
                    const data = await response.json();

                    const container = document.getElementById('logFileList');
                    container.innerHTML = '';

                    if (data.error) {
                        container.innerHTML = `<div class="log-file-card"><h3>Error</h3><p>${data.error}</p></div>`;
                        return;
                    }

                    if (data.files.length === 0) {
                        container.innerHTML = '<div class="log-file-card"><h3>No Log Files</h3><p>No log files found</p></div>';
                        return;
                    }

                    data.files.forEach(file => {
                        const card = document.createElement('div');
                        card.className = 'log-file-card';
                        card.onclick = () => selectLogFile(file.name);

                        const fileSize = (file.size / 1024).toFixed(1);
                        const modDate = new Date(file.modified * 1000).toLocaleString();

                        card.innerHTML = `
                            <h4>${file.name}</h4>
                            <p><strong>Size:</strong> ${fileSize} KB</p>
                            <p><strong>Modified:</strong> ${modDate}</p>
                        `;

                        container.appendChild(card);
                    });

                } catch (error) {
                    console.error('Failed to load log files:', error);
                    document.getElementById('logFileList').innerHTML = 
                        '<div class="log-file-card"><h3>Error</h3><p>Failed to load log files</p></div>';
                }
            }

            async function selectLogFile(filename) {
                // Update selected state
                document.querySelectorAll('.log-file-card').forEach(card => {
                    card.classList.remove('selected');
                });
                event.target.closest('.log-file-card').classList.add('selected');

                currentLogFile = filename;
                await refreshLogContent();
            }

            async function refreshLogContent() {
                if (!currentLogFile) return;

                try {
                    const response = await fetch(`/api/log_content/${currentLogFile}?max_lines=200`);
                    const data = await response.json();

                    const viewer = document.getElementById('logViewer');

                    if (data.error) {
                        viewer.textContent = `Error: ${data.error}`;
                        return;
                    }

                    const header = `=== ${data.filename} ===\\n` +
                                 `Total lines: ${data.total_lines} | File size: ${(data.file_size / 1024).toFixed(1)} KB\\n` +
                                 `Showing last ${data.lines.length} lines\\n\\n`;

                    viewer.textContent = header + data.lines.join('\\n');

                    // Scroll to bottom
                    viewer.scrollTop = viewer.scrollHeight;

                } catch (error) {
                    console.error('Failed to load log content:', error);
                    document.getElementById('logViewer').textContent = 'Failed to load log content.';
                }
            }

            // Auto-refresh functionality
            setInterval(loadStatus, 5000);  // Refresh processes every 5 seconds

            // Auto-refresh logs if a file is selected and logs tab is active
            setInterval(() => {
                const logsTabActive = document.getElementById('logs-tab').classList.contains('active');
                if (logsTabActive && currentLogFile) {
                    refreshLogContent();
                }
            }, 3000);  // Refresh logs every 3 seconds when viewing

            // Initial load
            async function initialize() {
                await loadLaunchConfigs();
                await loadStatus();
            }
            initialize();
        </script>
    </body>
    </html>
    """


@app.get("/api/status")
async def api_status():
    """Get status of all processes"""
    if not daemon:
        raise HTTPException(status_code=500, detail="Daemon not initialized")

    status = await daemon.get_status()
    return {name: info.dict() for name, info in status.items()}


@app.get("/api/launch_configs")
async def api_launch_configs():
    """Get available launch configurations"""
    if not daemon:
        raise HTTPException(status_code=500, detail="Daemon not initialized")

    return daemon.launch_configs


@app.get("/api/log_files")
async def api_get_log_files():
    """Get list of available log files"""
    if not daemon:
        raise HTTPException(status_code=500, detail="Daemon not initialized")

    return daemon.get_available_log_files()


@app.get("/api/log_content/{filename}")
async def api_get_log_content(filename: str, max_lines: int = 100):
    """Get content of a specific log file"""
    if not daemon:
        raise HTTPException(status_code=500, detail="Daemon not initialized")

    return daemon.get_log_file_content(filename, max_lines)


@app.post("/api/start/{process_name}")
async def api_start_process(process_name: str):
    """Start a specific process"""
    if not daemon:
        raise HTTPException(status_code=500, detail="Daemon not initialized")

    success = await daemon.start_process(process_name)
    if not success:
        raise HTTPException(status_code=500, detail=f"Failed to start {process_name}")

    return {"message": f"Started {process_name}"}


@app.post("/api/stop/{process_name}")  
async def api_stop_process(process_name: str):
    """Stop a specific process"""
    if not daemon:
        raise HTTPException(status_code=500, detail="Daemon not initialized")

    success = await daemon.stop_process(process_name)
    if not success:
        raise HTTPException(status_code=500, detail=f"Failed to stop {process_name}")

    return {"message": f"Stopped {process_name}"}


if __name__ == "__main__":
    import uvicorn

    # Get config file from environment or command line
    config_file = sys.argv[1] if len(sys.argv) > 1 else os.environ.get('PHYTO_ARM_CONFIG', '/configs/config.yaml')

    # Set environment variable for app startup
    os.environ['PHYTO_ARM_CONFIG'] = config_file

    logger.info(f"Starting PhytO-ARM daemon with config: {config_file}")

    uvicorn.run(
        app,
        host="0.0.0.0",
        port=8080,
        log_level="info"
    )
