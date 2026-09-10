"""ROS-independent contracts shared by the Qt UI and both backends."""
from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from typing import Dict, Mapping, Optional, Sequence


@dataclass(frozen=True)
class VelocityCommand:
    vx: float = 0.0
    vy: float = 0.0
    wz: float = 0.0

    @property
    def stopped(self) -> bool:
        return (
            abs(self.vx) < 1e-9
            and abs(self.vy) < 1e-9
            and abs(self.wz) < 1e-9
        )


@dataclass(frozen=True)
class BackendSnapshot:
    namespace: str
    cmd_vel_topic: str
    cmd_vel_subscribers: int
    control_state: Optional[int]
    power_enabled: Optional[bool]
    servo_enabled: Optional[bool]
    stream_enabled: Optional[bool]
    emo_active: Optional[bool]
    collision_active: Optional[bool]
    services_enabled: bool
    rby1_msgs_available: bool
    service_ready: Dict[str, bool]
    command: VelocityCommand
    command_stale: bool


class TaskCommandStatus(str, Enum):
    PENDING = "pending"
    SUCCEEDED = "succeeded"
    FAILED = "failed"
    CANCELED = "canceled"


@dataclass(frozen=True)
class TaskCommandState:
    status: TaskCommandStatus
    message: str = ""


@dataclass(frozen=True)
class TaskBackendState:
    """One consistent capture of the state needed by Task execution.

    Joint values and Cartesian RPY values use the same operator units shown
    in the UI: degrees and ``[m, m, m, deg, deg, deg]`` respectively.
    Timestamps are from ``time.monotonic()``.
    """

    captured_at: float
    robot_state_updated_at: Optional[float]
    control_state: Optional[int]
    stream_enabled: Optional[bool]
    emo_active: Optional[bool]
    collision_active: Optional[bool]
    motion_active: bool
    joint_groups: Mapping[str, Optional[Sequence[float]]]
    joint_updated_at: Mapping[str, Optional[float]]
    joint_order_verified: Mapping[str, bool]
    cartesian: Mapping[str, Optional[Sequence[float]]]
    cartesian_updated_at: Mapping[str, Optional[float]]
    driver_safety_verified: bool
    driver_safety_updated_at: Optional[float]
