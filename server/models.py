"""
Data models and enums for PhytO-ARM server
"""

from datetime import datetime
from enum import Enum
from typing import List, Optional

try:
    from pydantic.v1 import BaseModel
except ImportError:
    from pydantic import BaseModel


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
