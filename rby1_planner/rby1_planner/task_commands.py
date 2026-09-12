"""Planner Task authoring with execution-time perception steps.

Ordinary motion commands remain the canonical :mod:`rby1_control` value
objects. Camera commands are planner-only descriptors and are resolved to a
canonical Cartesian command before anything is sent to the control backend.
"""
from __future__ import annotations

from dataclasses import dataclass, replace
import math
from typing import (
    TYPE_CHECKING,
    Iterable,
    Optional,
    Sequence,
    Union,
    cast,
)

from rby1_control.task_commands import (
    BODY_JOINT_GROUPS,
    CARTESIAN_ARMS,
    JOINT_GROUP_DOF,
    CommandKind,
    Task as _CanonicalTask,
    TaskCommand,
    TaskDefinition,
    list_sum,
)

from .observation import (
    CartesianTarget,
    ObjectObservation,
    compose_pose_right,
    normalize_quaternion,
    observation_to_cartesian_target,
)


if TYPE_CHECKING:
    from .camera import CameraClient


def _finite_number(
    value: object,
    label: str,
    *,
    positive: bool = False,
) -> float:
    if isinstance(value, bool):
        raise ValueError(f'{label} must be a finite number')
    try:
        result = float(value)
    except (TypeError, ValueError) as exc:
        raise ValueError(f'{label} must be a finite number') from exc
    if not math.isfinite(result) or (positive and result <= 0.0):
        qualifier = 'positive ' if positive else ''
        raise ValueError(f'{label} must be a {qualifier}finite number')
    return result


def _finite_values(
    values: Sequence[object],
    count: int,
    label: str,
) -> tuple[float, ...]:
    if isinstance(values, (str, bytes)):
        raise ValueError(f'{label} must contain {count} finite numbers')
    result = tuple(_finite_number(value, label) for value in values)
    if len(result) != count:
        raise ValueError(f'{label} must contain {count} finite numbers')
    return result


@dataclass(frozen=True)
class CameraLinearAbsoluteStep:
    """Deferred object lookup followed by a canonical Cartesian motion."""

    group: str
    object_id: str
    minimum_time: float
    linear_velocity: float
    angular_velocity: float
    acceleration_scaling: float
    detection_timeout_sec: float = 3.0
    max_age_sec: Optional[float] = None
    minimum_confidence: Optional[float] = None
    object_to_end_effector_position: tuple[float, ...] = (0.0, 0.0, 0.0)
    object_to_end_effector_orientation_xyzw: tuple[float, ...] = (
        0.0,
        0.0,
        0.0,
        1.0,
    )

    def __post_init__(self) -> None:
        group = str(self.group).strip()
        object_id = str(self.object_id).strip()
        if not object_id:
            raise ValueError('object_id must not be empty')

        minimum_time = _finite_number(
            self.minimum_time,
            'minimum_time',
            positive=True,
        )
        linear_velocity = _finite_number(
            self.linear_velocity,
            'linear_velocity',
            positive=True,
        )
        angular_velocity = _finite_number(
            self.angular_velocity,
            'angular_velocity',
            positive=True,
        )
        acceleration_scaling = _finite_number(
            self.acceleration_scaling,
            'acceleration_scaling',
            positive=True,
        )
        detection_timeout_sec = _finite_number(
            self.detection_timeout_sec,
            'detection_timeout_sec',
            positive=True,
        )
        max_age_sec = (
            None
            if self.max_age_sec is None
            else _finite_number(
                self.max_age_sec,
                'max_age_sec',
                positive=True,
            )
        )
        minimum_confidence = (
            None
            if self.minimum_confidence is None
            else _finite_number(
                self.minimum_confidence,
                'minimum_confidence',
            )
        )
        if (
            minimum_confidence is not None
            and not 0.0 <= minimum_confidence <= 1.0
        ):
            raise ValueError(
                'minimum_confidence must be in the range [0, 1]'
            )
        offset_position = _finite_values(
            self.object_to_end_effector_position,
            3,
            'object_to_end_effector_position',
        )
        offset_orientation = normalize_quaternion(
            self.object_to_end_effector_orientation_xyzw
        )

        # Reuse the canonical command validation for arm and motion limits.
        TaskCommand(
            kind=CommandKind.LINEAR_ABSOLUTE,
            group=group,
            values=(0.0,) * 6,
            minimum_time=minimum_time,
            linear_velocity=linear_velocity,
            angular_velocity=angular_velocity,
            acceleration_scaling=acceleration_scaling,
        )

        object.__setattr__(self, 'group', group)
        object.__setattr__(self, 'object_id', object_id)
        object.__setattr__(self, 'minimum_time', minimum_time)
        object.__setattr__(self, 'linear_velocity', linear_velocity)
        object.__setattr__(self, 'angular_velocity', angular_velocity)
        object.__setattr__(
            self,
            'acceleration_scaling',
            acceleration_scaling,
        )
        object.__setattr__(
            self,
            'detection_timeout_sec',
            detection_timeout_sec,
        )
        object.__setattr__(self, 'max_age_sec', max_age_sec)
        object.__setattr__(
            self,
            'minimum_confidence',
            minimum_confidence,
        )
        object.__setattr__(
            self,
            'object_to_end_effector_position',
            offset_position,
        )
        object.__setattr__(
            self,
            'object_to_end_effector_orientation_xyzw',
            offset_orientation,
        )

    def resolve(self, values: Sequence[object]) -> TaskCommand:
        """Create the canonical command sent to the control backend."""

        return TaskCommand(
            kind=CommandKind.LINEAR_ABSOLUTE,
            group=self.group,
            values=tuple(values),
            minimum_time=self.minimum_time,
            linear_velocity=self.linear_velocity,
            angular_velocity=self.angular_velocity,
            acceleration_scaling=self.acceleration_scaling,
        )

    def resolve_observation(
        self,
        observation: ObjectObservation,
    ) -> TaskCommand:
        """Compose the configured object-to-EE transform and resolve it."""

        if not isinstance(observation, ObjectObservation):
            raise TypeError('camera must return an ObjectObservation')
        position, orientation = compose_pose_right(
            observation.position,
            observation.orientation_xyzw,
            self.object_to_end_effector_position,
            self.object_to_end_effector_orientation_xyzw,
        )
        target = observation_to_cartesian_target(
            replace(
                observation,
                position=position,
                orientation_xyzw=orientation,
            ),
            target_frame='base',
        )
        return self.resolve(target)


DynamicTaskStep = Union[TaskCommand, CameraLinearAbsoluteStep]


@dataclass(frozen=True)
class DynamicTaskDefinition:
    """Immutable planner Task containing at least one deferred command."""

    name: str
    commands: tuple[DynamicTaskStep, ...]
    description: str = ''

    def __post_init__(self) -> None:
        if not isinstance(self.name, str) or not self.name.strip():
            raise ValueError('Task name must be nonempty')
        if not isinstance(self.description, str):
            raise ValueError('Task description must be a string')
        commands = tuple(self.commands)
        if not all(
            isinstance(command, (TaskCommand, CameraLinearAbsoluteStep))
            for command in commands
        ):
            raise ValueError('Planner Task commands are invalid')
        if not any(
            isinstance(command, CameraLinearAbsoluteStep)
            for command in commands
        ):
            raise ValueError(
                'DynamicTaskDefinition requires a deferred camera command'
            )
        object.__setattr__(self, 'commands', commands)


RunnableTaskDefinition = Union[TaskDefinition, DynamicTaskDefinition]


class Task(_CanonicalTask):
    """Canonical Task builder extended with deferred camera lookup."""

    def camera_linear_absolute(
        self,
        arm: str,
        object_id: str,
        tcp_motion: Sequence[object],
        *,
        detection_timeout_sec: float = 3.0,
        max_age_sec: Optional[float] = None,
        minimum_confidence: Optional[float] = None,
        object_to_end_effector_position: Sequence[object] = (
            0.0,
            0.0,
            0.0,
        ),
        object_to_end_effector_orientation_xyzw: Sequence[object] = (
            0.0,
            0.0,
            0.0,
            1.0,
        ),
    ) -> 'Task':
        """Add a Cartesian target that will be captured during execution.

        The identity offset targets the tag pose exactly. Supply a calibrated
        object-to-end-effector transform for a real approach or grasp.
        """

        probe = _CanonicalTask('_camera_command_validation')
        probe.linear_absolute(arm, (0.0,) * 6, tcp_motion)
        motion = probe.task_list[0]
        self.task_list.append(CameraLinearAbsoluteStep(
            group=str(motion.group),
            object_id=object_id,
            minimum_time=float(motion.minimum_time),
            linear_velocity=float(motion.linear_velocity),
            angular_velocity=float(motion.angular_velocity),
            acceleration_scaling=float(motion.acceleration_scaling),
            detection_timeout_sec=detection_timeout_sec,
            max_age_sec=max_age_sec,
            minimum_confidence=minimum_confidence,
            object_to_end_effector_position=tuple(
                object_to_end_effector_position
            ),
            object_to_end_effector_orientation_xyzw=tuple(
                object_to_end_effector_orientation_xyzw
            ),
        ))
        return self

    def extend(
        self,
        other: Union[
            _CanonicalTask,
            TaskDefinition,
            DynamicTaskDefinition,
            Iterable[DynamicTaskStep],
        ],
    ) -> 'Task':
        if isinstance(other, _CanonicalTask):
            commands = other.task_list
        elif isinstance(other, (TaskDefinition, DynamicTaskDefinition)):
            commands = other.commands
        else:
            commands = list(other)
        if not all(
            isinstance(command, (TaskCommand, CameraLinearAbsoluteStep))
            for command in commands
        ):
            raise ValueError('extend accepts only planner Task commands')
        self.task_list.extend(commands)
        return self

    def build(self) -> RunnableTaskDefinition:
        commands = tuple(self.task_list)
        if any(
            isinstance(command, CameraLinearAbsoluteStep)
            for command in commands
        ):
            return DynamicTaskDefinition(
                name=self.name,
                description=self.description,
                commands=commands,
            )
        return TaskDefinition(
            name=self.name,
            description=self.description,
            commands=commands,
        )


def get_object_position(
    camera: 'CameraClient',
    object_id: str = 'tag_0',
    *,
    max_age_sec: Optional[float] = None,
    minimum_confidence: Optional[float] = None,
    newer_than_ns: Optional[int] = None,
) -> CartesianTarget:
    """Return one fresh, Task-ready Cartesian object-position snapshot.

    The returned order and units match ``Task.linear_absolute`` exactly:
    ``(x_m, y_m, z_m, roll_deg, pitch_deg, yaw_deg)``. This helper never waits
    for perception; ``CameraClient`` raises ``ObservationUnavailable`` when a
    fresh detection or its TF is not currently available.
    """

    getter = getattr(camera, 'require_object_position', None)
    if not callable(getter):
        raise TypeError(
            'camera must provide require_object_position(object_id, ...)'
        )

    # TaskCommand does not carry a reference-frame field; the control backend
    # executes LINEAR_ABSOLUTE in base. Never let a camera-frame value cross
    # this boundary looking like a base-frame command.
    query: dict[str, object] = {'target_frame': 'base'}
    if max_age_sec is not None:
        query['max_age_sec'] = max_age_sec
    if minimum_confidence is not None:
        query['minimum_confidence'] = minimum_confidence
    if newer_than_ns is not None:
        query['newer_than_ns'] = newer_than_ns

    values = getter(object_id, **query)
    if isinstance(values, (str, bytes)):
        raise ValueError('camera Cartesian position must contain 6 numbers')
    try:
        result = tuple(float(value) for value in values)
    except (TypeError, ValueError) as exc:
        raise ValueError(
            'camera Cartesian position must contain 6 finite numbers'
        ) from exc
    if len(result) != 6 or not all(math.isfinite(value) for value in result):
        raise ValueError(
            'camera Cartesian position must contain 6 finite numbers'
        )
    return cast(CartesianTarget, result)


__all__ = [
    'BODY_JOINT_GROUPS',
    'CARTESIAN_ARMS',
    'CameraLinearAbsoluteStep',
    'JOINT_GROUP_DOF',
    'CommandKind',
    'DynamicTaskDefinition',
    'DynamicTaskStep',
    'RunnableTaskDefinition',
    'Task',
    'TaskCommand',
    'TaskDefinition',
    'get_object_position',
    'list_sum',
]
