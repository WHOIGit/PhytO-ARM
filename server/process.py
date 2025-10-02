"""
PhytO-ARM process management
"""

import logging
import subprocess
from datetime import datetime
from typing import Dict, Optional

from .models import ProcessInfo, ProcessState


logger = logging.getLogger(__name__)


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
            logger.warning("%s is already running", self.name)
            return True

        try:
            self.info.state = ProcessState.STARTING
            logger.info("Starting %s: %s", self.name, self.command)

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

            logger.info("%s started with PID %s", self.name, self.info.pid)
            return True

        except (subprocess.SubprocessError, OSError) as e:
            logger.error("Failed to start %s: %s", self.name, e)
            self.info.state = ProcessState.FAILED
            return False

    def stop(self) -> bool:
        """Stop the process"""
        if not self.is_running():
            logger.warning("%s is not running", self.name)
            return True

        try:
            self.info.state = ProcessState.STOPPING
            logger.info("Stopping %s (PID %s)", self.name, self.info.pid)

            self.process.terminate()
            try:
                exit_code = self.process.wait(timeout=5.0)
                self.info.exit_code = exit_code
            except subprocess.TimeoutExpired:
                if self.dont_kill:
                    logger.warning("Failed to terminate %s, but refusing to kill", self.name)
                else:
                    logger.warning("Failed to terminate %s, killing", self.name)
                    self.process.kill()
                    exit_code = self.process.wait()
                    self.info.exit_code = exit_code

            self.info.stopped_at = datetime.now()
            self.info.state = ProcessState.STOPPED
            self.process = None

            logger.info("%s stopped with exit code %s", self.name, self.info.exit_code)
            return True

        except (subprocess.SubprocessError, OSError) as e:
            logger.error("Failed to stop %s: %s", self.name, e)
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
