"""Small IDE-authored Task language for RB-Y1.

Task source uses operator-friendly units:
- joint positions: degrees
- Cartesian positions: metres
- Cartesian orientation: roll/pitch/yaw degrees in the base frame
- velocity/acceleration fields: the native rby1_msgs units
"""
from __future__ import annotations

from dataclasses import asdict, dataclass
from enum import Enum
import json
import math
import os
from pathlib import Path
import tempfile
from typing import Dict, Iterable, Mapping, Optional, Sequence, Tuple, Union


JOINT_GROUP_DOF = {
    "right_arm": 7,
    "left_arm": 7,
    "torso": 6,
    "head": 2,
}
CARTESIAN_ARMS = ("right_arm", "left_arm")
BODY_JOINT_GROUPS = ("torso", "right_arm", "left_arm")


class CommandKind(str, Enum):
    JOINT_ABSOLUTE = "joint_absolute"
    JOINT_ABSOLUTE_MULTI = "joint_absolute_multi"
    JOINT_RELATIVE = "joint_relative"
    LINEAR_ABSOLUTE = "linear_absolute"
    LINEAR_RELATIVE = "linear_relative"
    BASE_VELOCITY = "base_velocity"
    DELAY = "delay"


def _number(value: object, label: str, *, positive: bool = False) -> float:
    if isinstance(value, bool):
        raise ValueError(f"{label} must be a finite number")
    try:
        result = float(value)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{label} must be a finite number") from exc
    if not math.isfinite(result) or (positive and result <= 0.0):
        qualifier = "positive " if positive else ""
        raise ValueError(f"{label} must be a {qualifier}finite number")
    return result


def _values(values: Sequence[object], count: int, label: str) -> Tuple[float, ...]:
    if isinstance(values, (str, bytes)):
        raise ValueError(f"{label} must contain {count} numeric values")
    result = tuple(_number(value, label) for value in values)
    if len(result) != count:
        raise ValueError(f"{label} requires {count} values; got {len(result)}")
    return result


def list_sum(left: Sequence[object], right: Sequence[object]) -> list[float]:
    """Element-wise addition with strict length and numeric validation."""

    if len(left) != len(right):
        raise ValueError("list_sum operands must have the same length")
    return [
        _number(a, "list_sum value") + _number(b, "list_sum value")
        for a, b in zip(left, right)
    ]


@dataclass(frozen=True)
class TaskCommand:
    kind: CommandKind
    group: Optional[str] = None
    values: Tuple[float, ...] = ()
    joint_targets: Tuple[Tuple[str, Tuple[float, ...]], ...] = ()
    minimum_time: Optional[float] = None
    velocity_limit: Optional[float] = None
    acceleration_limit: Optional[float] = None
    linear_velocity: Optional[float] = None
    angular_velocity: Optional[float] = None
    acceleration_scaling: Optional[float] = None
    seconds: Optional[float] = None

    def __post_init__(self) -> None:
        if not isinstance(self.kind, CommandKind):
            raise ValueError("Task command kind is invalid")

        if self.kind is CommandKind.DELAY:
            _number(self.seconds, "seconds", positive=True)
            if (
                self.group is not None
                or self.values
                or self.joint_targets
                or any(value is not None for value in (
                    self.minimum_time,
                    self.velocity_limit,
                    self.acceleration_limit,
                    self.linear_velocity,
                    self.angular_velocity,
                    self.acceleration_scaling,
                ))
            ):
                raise ValueError("delay cannot contain a motion target")
            return

        if self.kind is CommandKind.BASE_VELOCITY:
            if self.group != "base":
                raise ValueError("base velocity command group must be 'base'")

            _values(self.values, 3, "base velocity")
            _number(self.seconds, "seconds", positive=True)

            if self.joint_targets:
                raise ValueError(
                    "base velocity command cannot contain joint targets"
            )

            if any(value is not None for value in (
                self.minimum_time,
                self.velocity_limit,
                self.acceleration_limit,
                self.linear_velocity,
                self.angular_velocity,
                self.acceleration_scaling,
            )):
                raise ValueError(
                    "base velocity command contains incompatible fields"
                )

            return

        if (
            self.kind is not CommandKind.JOINT_ABSOLUTE_MULTI
            and self.group is None
        ):
            raise ValueError("motion command group is missing")
        _number(self.minimum_time, "minimum_time", positive=True)

        if self.kind in (CommandKind.JOINT_ABSOLUTE, CommandKind.JOINT_RELATIVE):
            if self.joint_targets:
                raise ValueError(
                    "single-group joint command contains multiple targets"
                )
            dof = JOINT_GROUP_DOF.get(self.group)
            if dof is None:
                raise ValueError(f"unknown joint group: {self.group}")
            _values(self.values, dof, f"{self.group} target")
            _number(self.velocity_limit, "velocity_limit", positive=True)
            _number(self.acceleration_limit, "acceleration_limit", positive=True)
            if any(value is not None for value in (
                self.linear_velocity,
                self.angular_velocity,
                self.acceleration_scaling,
                self.seconds,
            )):
                raise ValueError("joint command contains Cartesian/delay fields")
            return

        if self.kind is CommandKind.JOINT_ABSOLUTE_MULTI:
            if self.group is not None or self.values:
                raise ValueError("multi-group joint command contains a single target")
            if (
                not isinstance(self.joint_targets, tuple)
                or len(self.joint_targets) < 2
            ):
                raise ValueError(
                    "multi-group joint command requires at least two targets"
                )
            seen = set()
            for item in self.joint_targets:
                if not isinstance(item, tuple) or len(item) != 2:
                    raise ValueError("multi-group joint target is invalid")
                group, values = item
                if group not in BODY_JOINT_GROUPS:
                    raise ValueError(
                        f"unsupported multi-group joint target: {group}"
                    )
                if group in seen:
                    raise ValueError(
                        f"duplicate multi-group joint target: {group}"
                    )
                seen.add(group)
                _values(values, JOINT_GROUP_DOF[group], f"{group} target")
            _number(self.velocity_limit, "velocity_limit", positive=True)
            _number(self.acceleration_limit, "acceleration_limit", positive=True)
            if any(value is not None for value in (
                self.linear_velocity,
                self.angular_velocity,
                self.acceleration_scaling,
                self.seconds,
            )):
                raise ValueError(
                    "multi-group joint command contains Cartesian/delay fields"
                )
            return

        if self.group not in CARTESIAN_ARMS:
            raise ValueError(f"unknown Cartesian arm: {self.group}")
        if self.joint_targets:
            raise ValueError("Cartesian command contains joint targets")
        _values(self.values, 6, f"{self.group} TCP target")
        _number(self.linear_velocity, "linear_velocity", positive=True)
        _number(self.angular_velocity, "angular_velocity", positive=True)
        scaling = _number(
            self.acceleration_scaling,
            "acceleration_scaling",
            positive=True,
        )
        if scaling > 1.0:
            raise ValueError("acceleration_scaling cannot exceed 1.0")
        if any(value is not None for value in (
            self.velocity_limit,
            self.acceleration_limit,
            self.seconds,
        )):
            raise ValueError("Cartesian command contains joint/delay fields")

    @property
    def timeout_seconds(self) -> float:
        if self.kind is CommandKind.DELAY:
            return float(self.seconds or 0.0) + 1.0
        # Fallback for callers that do not have a fresh robot state.  The
        # TaskRunner refines this using the actual travel distance and the
        # command's velocity limits.
        return max(10.0, float(self.minimum_time or 0.0) * 3.0 + 5.0)


@dataclass(frozen=True)
class TaskDefinition:
    name: str
    commands: Tuple[TaskCommand, ...]
    description: str = ""

    def __post_init__(self) -> None:
        if not isinstance(self.name, str) or not self.name.strip():
            raise ValueError("Task name must be nonempty")
        if not isinstance(self.description, str):
            raise ValueError("Task description must be a string")
        if not all(isinstance(command, TaskCommand) for command in self.commands):
            raise ValueError("Task commands are invalid")


class Task:
    """Mutable authoring helper; ``build()`` creates an immutable Task."""

    def __init__(self, name: str, description: str = "") -> None:
        self.name = str(name)
        self.description = str(description)
        self.task_list: list[TaskCommand] = []

    def joint_absolute(
        self,
        group: str,
        positions: Sequence[object],
        joint_motion: Sequence[object],
    ) -> "Task":
        return self._joint(
            CommandKind.JOINT_ABSOLUTE,
            group,
            positions,
            joint_motion,
        )

    def joint_relative(
        self,
        group: str,
        offsets: Sequence[object],
        joint_motion: Sequence[object],
    ) -> "Task":
        return self._joint(
            CommandKind.JOINT_RELATIVE,
            group,
            offsets,
            joint_motion,
        )

    def joint_absolute_multi(
        self,
        targets: Mapping[str, Sequence[object]],
        joint_motion: Sequence[object],
    ) -> "Task":
        """Move two or more body joint groups in one synchronized command."""

        if not isinstance(targets, Mapping) or len(targets) < 2:
            raise ValueError("targets must contain at least two joint groups")
        minimum_time, velocity_limit, acceleration_limit = _values(
            joint_motion,
            3,
            "Joint motion settings",
        )
        normalized = []
        for group, positions in targets.items():
            if group not in BODY_JOINT_GROUPS:
                raise ValueError(
                    f"unsupported multi-group joint target: {group}"
                )
            normalized.append((
                group,
                _values(positions, JOINT_GROUP_DOF[group], f"{group} target"),
            ))
        self.task_list.append(TaskCommand(
            kind=CommandKind.JOINT_ABSOLUTE_MULTI,
            joint_targets=tuple(normalized),
            minimum_time=_number(minimum_time, "minimum_time", positive=True),
            velocity_limit=_number(
                velocity_limit,
                "velocity_limit",
                positive=True,
            ),
            acceleration_limit=_number(
                acceleration_limit,
                "acceleration_limit",
                positive=True,
            ),
        ))
        return self

    def whole_body_joint_absolute(
        self,
        *,
        torso: Sequence[object],
        right_arm: Sequence[object],
        left_arm: Sequence[object],
        joint_motion: Sequence[object],
    ) -> "Task":
        """Move torso and both arms together in one synchronized command."""

        return self.joint_absolute_multi(
            {
                "torso": torso,
                "right_arm": right_arm,
                "left_arm": left_arm,
            },
            joint_motion,
        )

    def _joint(
        self,
        kind: CommandKind,
        group: str,
        values: Sequence[object],
        joint_motion: Sequence[object],
    ) -> "Task":
        dof = JOINT_GROUP_DOF.get(group)
        if dof is None:
            raise ValueError(f"unknown joint group: {group}")
        minimum_time, velocity_limit, acceleration_limit = _values(
            joint_motion,
            3,
            "Joint motion settings",
        )
        self.task_list.append(TaskCommand(
            kind=kind,
            group=group,
            values=_values(values, dof, f"{group} target"),
            minimum_time=_number(minimum_time, "minimum_time", positive=True),
            velocity_limit=_number(velocity_limit, "velocity_limit", positive=True),
            acceleration_limit=_number(
                acceleration_limit,
                "acceleration_limit",
                positive=True,
            ),
        ))
        return self

    def linear_absolute(
        self,
        arm: str,
        pose: Sequence[object],
        tcp_motion: Sequence[object],
    ) -> "Task":
        return self._linear(
            CommandKind.LINEAR_ABSOLUTE,
            arm,
            pose,
            tcp_motion,
        )

    def linear_relative(
        self,
        arm: str,
        offset: Sequence[object],
        tcp_motion: Sequence[object],
    ) -> "Task":
        return self._linear(
            CommandKind.LINEAR_RELATIVE,
            arm,
            offset,
            tcp_motion,
        )

    def _linear(
        self,
        kind: CommandKind,
        arm: str,
        values: Sequence[object],
        tcp_motion: Sequence[object],
    ) -> "Task":
        (
            minimum_time,
            linear_velocity,
            angular_velocity,
            acceleration_scaling,
        ) = _values(tcp_motion, 4, "TCP motion settings")
        self.task_list.append(TaskCommand(
            kind=kind,
            group=arm,
            values=_values(values, 6, f"{arm} TCP target"),
            minimum_time=_number(minimum_time, "minimum_time", positive=True),
            linear_velocity=_number(
                linear_velocity,
                "linear_velocity",
                positive=True,
            ),
            angular_velocity=_number(
                angular_velocity,
                "angular_velocity",
                positive=True,
            ),
            acceleration_scaling=_number(
                acceleration_scaling,
                "acceleration_scaling",
                positive=True,
            ),
        ))
        return self
    def base_velocity(
            self,
            vx: float,
            vy: float,
            wz: float,
            seconds: float,
    ) -> "Task":
        self.task_list.append(TaskCommand(
            kind=CommandKind.BASE_VELOCITY,
            group="base",
            values=(
                _number(vx, "vx"),
                _number(vy, "vy"),
                _number(wz, "wz"),
            ),
            seconds=_number(seconds, "seconds", positive=True),
        ))

        return self

    def delay(self, milliseconds: float) -> "Task":
        seconds = _number(milliseconds, "milliseconds", positive=True) / 1000.0
        self.task_list.append(TaskCommand(kind=CommandKind.DELAY, seconds=seconds))
        return self

    def extend(
        self,
        other: Union["Task", TaskDefinition, Iterable[TaskCommand]],
    ) -> "Task":
        if isinstance(other, Task):
            commands = other.task_list
        elif isinstance(other, TaskDefinition):
            commands = other.commands
        else:
            commands = list(other)
        if not all(isinstance(command, TaskCommand) for command in commands):
            raise ValueError("extend accepts only Task commands")
        self.task_list.extend(commands)
        return self

    def build(self) -> TaskDefinition:
        return TaskDefinition(
            name=self.name,
            description=self.description,
            commands=tuple(self.task_list),
        )


def default_catalog_path() -> Path:
    override = os.environ.get("RBY1_TASK_CATALOG")
    if override:
        return Path(override).expanduser()
    config_home = os.environ.get("XDG_CONFIG_HOME")
    root = Path(config_home).expanduser() if config_home else Path.home() / ".config"
    return root / "rby1_control" / "task_catalog.json"


def _command_dict(command: TaskCommand) -> dict:
    payload = asdict(command)
    payload["kind"] = command.kind.value
    payload["values"] = list(command.values)
    if command.joint_targets:
        payload["joint_targets"] = {
            group: list(values)
            for group, values in command.joint_targets
        }
    else:
        payload.pop("joint_targets", None)
    return {key: value for key, value in payload.items() if value is not None}


def save_catalog(
        tasks: Mapping[str, TaskDefinition], path: Optional[Path] = None) -> Path:
    target = Path(path) if path is not None else default_catalog_path()
    target.parent.mkdir(parents=True, exist_ok=True)
    payload = {
        "schema_version": 1,
        "tasks": [
            {
                "name": task.name,
                "description": task.description,
                "commands": [_command_dict(command) for command in task.commands],
            }
            for _, task in sorted(tasks.items())
        ],
    }
    descriptor, temporary_name = tempfile.mkstemp(
        prefix=f".{target.name}.", suffix=".tmp", dir=str(target.parent)
    )
    try:
        with os.fdopen(descriptor, "w", encoding="utf-8") as stream:
            json.dump(payload, stream, ensure_ascii=False, indent=2)
            stream.write("\n")
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary_name, target)
    except Exception:
        try:
            os.unlink(temporary_name)
        except FileNotFoundError:
            pass
        raise
    return target


def load_catalog(path: Optional[Path] = None) -> Dict[str, TaskDefinition]:
    target = Path(path) if path is not None else default_catalog_path()
    with target.open("r", encoding="utf-8") as stream:
        payload = json.load(stream)
    if not isinstance(payload, dict) or payload.get("schema_version") != 1:
        raise ValueError("unsupported Task catalog schema")
    raw_tasks = payload.get("tasks")
    if not isinstance(raw_tasks, list):
        raise ValueError("Task catalog tasks must be a list")
    tasks: Dict[str, TaskDefinition] = {}
    for raw_task in raw_tasks:
        if not isinstance(raw_task, dict):
            raise ValueError("Task catalog entry must be an object")
        commands = []
        for raw_command in raw_task.get("commands", []):
            if not isinstance(raw_command, dict):
                raise ValueError("Task command must be an object")
            item = dict(raw_command)
            item["kind"] = CommandKind(item["kind"])
            item["values"] = tuple(item.get("values", ()))
            raw_joint_targets = item.get("joint_targets", {})
            if not isinstance(raw_joint_targets, dict):
                raise ValueError("Task multi-group joint targets must be an object")
            item["joint_targets"] = tuple(
                (group, tuple(values))
                for group, values in raw_joint_targets.items()
            )
            commands.append(TaskCommand(**item))
        task = TaskDefinition(
            name=raw_task.get("name", ""),
            description=raw_task.get("description", ""),
            commands=tuple(commands),
        )
        if task.name in tasks:
            raise ValueError(f"duplicate Task name: {task.name}")
        tasks[task.name] = task
    return tasks


def delete_catalog_task(name: str, path: Optional[Path] = None) -> Dict[str, TaskDefinition]:
    tasks = load_catalog(path)
    if name not in tasks:
        raise KeyError(name)
    del tasks[name]
    save_catalog(tasks, path)
    return tasks


def compile_task_source(path: Optional[Path] = None) -> Dict[str, TaskDefinition]:
    """Reload ``task.py``, validate every Task and atomically save the catalog."""

    import importlib
    from . import task as task_source

    module = importlib.reload(task_source)
    authored = module.build_tasks()
    if not isinstance(authored, Mapping):
        raise ValueError("build_tasks() must return a name-to-Task mapping")
    result: Dict[str, TaskDefinition] = {}
    for name, value in authored.items():
        if not isinstance(name, str) or not name:
            raise ValueError("Task registry names must be nonempty strings")
        if isinstance(value, Task):
            definition = value.build()
        elif isinstance(value, TaskDefinition):
            definition = value
        else:
            raise ValueError(f"Task '{name}' is not a Task")
        if name != definition.name:
            raise ValueError(f"Task registry key '{name}' does not match '{definition.name}'")
        if name in result:
            raise ValueError(f"duplicate Task name: {name}")
        result[name] = definition
    save_catalog(result, path)
    return result
