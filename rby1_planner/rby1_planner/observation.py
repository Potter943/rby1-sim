"""ROS-independent object-observation value types and pose mathematics."""
from __future__ import annotations

from dataclasses import dataclass, replace
import math
from typing import Iterable, Tuple


Vector3 = Tuple[float, float, float]
Quaternion = Tuple[float, float, float, float]
RollPitchYawDegrees = Tuple[float, float, float]
CartesianTarget = Tuple[float, float, float, float, float, float]
PoseTuple = Tuple[Vector3, Quaternion]


def _finite_tuple(values: Iterable[object], count: int, label: str) -> tuple:
    result = []
    for value in values:
        if isinstance(value, bool):
            raise ValueError(f'{label} must contain finite numbers')
        try:
            number = float(value)
        except (TypeError, ValueError) as exc:
            raise ValueError(f'{label} must contain finite numbers') from exc
        if not math.isfinite(number):
            raise ValueError(f'{label} must contain finite numbers')
        result.append(number)
    if len(result) != count:
        raise ValueError(f'{label} must contain exactly {count} values')
    return tuple(result)


def normalize_quaternion(
    values: Iterable[object],
    *,
    norm_tolerance: float = 1.0e-3,
) -> Quaternion:
    """Validate and normalize an x/y/z/w quaternion."""

    if isinstance(norm_tolerance, bool):
        raise ValueError('norm_tolerance must be a nonnegative finite number')
    tolerance = float(norm_tolerance)
    if not math.isfinite(tolerance) or tolerance < 0.0:
        raise ValueError('norm_tolerance must be a nonnegative finite number')
    quaternion = _finite_tuple(values, 4, 'orientation_xyzw')
    norm = math.sqrt(sum(value * value for value in quaternion))
    if norm <= 1.0e-12:
        raise ValueError('orientation_xyzw must not be a zero quaternion')
    if abs(norm - 1.0) > tolerance:
        raise ValueError('orientation_xyzw must be normalized')
    return tuple(value / norm for value in quaternion)  # type: ignore[return-value]


@dataclass(frozen=True)
class ObjectObservation:
    """One deeply immutable, validated object pose observation.

    ``stamp_ns`` and ``received_at_ns`` use the ROS clock associated with the
    owning node. Position is metres and orientation is x/y/z/w.
    """

    object_id: str
    frame_id: str
    stamp_ns: int
    received_at_ns: int
    position: Vector3
    orientation_xyzw: Quaternion
    confidence: float

    def __post_init__(self) -> None:
        object_id = str(self.object_id).strip()
        frame_id = str(self.frame_id).strip()
        if not object_id:
            raise ValueError('object_id must not be empty')
        if not frame_id:
            raise ValueError('frame_id must not be empty')
        if isinstance(self.stamp_ns, bool) or int(self.stamp_ns) < 0:
            raise ValueError('stamp_ns must be nonnegative')
        if (
            isinstance(self.received_at_ns, bool)
            or int(self.received_at_ns) < 0
        ):
            raise ValueError('received_at_ns must be nonnegative')

        position = _finite_tuple(self.position, 3, 'position')
        quaternion = normalize_quaternion(self.orientation_xyzw)
        if isinstance(self.confidence, bool):
            raise ValueError('confidence must be in the range [0.0, 1.0]')
        confidence = float(self.confidence)
        if not math.isfinite(confidence) or not 0.0 <= confidence <= 1.0:
            raise ValueError('confidence must be in the range [0.0, 1.0]')

        object.__setattr__(self, 'object_id', object_id)
        object.__setattr__(self, 'frame_id', frame_id)
        object.__setattr__(self, 'stamp_ns', int(self.stamp_ns))
        object.__setattr__(self, 'received_at_ns', int(self.received_at_ns))
        object.__setattr__(self, 'position', position)
        object.__setattr__(self, 'orientation_xyzw', quaternion)
        object.__setattr__(self, 'confidence', confidence)

    def age_seconds(self, now_ns: int) -> float:
        return (int(now_ns) - self.stamp_ns) / 1_000_000_000.0


def quaternion_multiply(left: Quaternion, right: Quaternion) -> Quaternion:
    """Compose two normalized x/y/z/w quaternions."""

    lx, ly, lz, lw = normalize_quaternion(left)
    rx, ry, rz, rw = normalize_quaternion(right)
    return normalize_quaternion((
        lw * rx + lx * rw + ly * rz - lz * ry,
        lw * ry - lx * rz + ly * rw + lz * rx,
        lw * rz + lx * ry - ly * rx + lz * rw,
        lw * rw - lx * rx - ly * ry - lz * rz,
    ))


def quaternion_to_rpy_deg(
    orientation_xyzw: Iterable[object],
) -> RollPitchYawDegrees:
    """Convert normalized ROS x/y/z/w orientation to backend RPY degrees.

    This intentionally uses the same fixed-axis roll/pitch/yaw convention as
    ``Rby1ControlNode._quaternion_to_rpy_deg``.  Clamping the pitch input keeps
    floating-point noise at the +/-90 degree gimbal boundary inside the domain
    accepted by ``asin``.
    """

    x, y, z, w = normalize_quaternion(orientation_xyzw)

    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (w * y - z * x)
    sinp = max(-1.0, min(1.0, sinp))
    pitch = math.asin(sinp)

    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return (
        math.degrees(roll),
        math.degrees(pitch),
        math.degrees(yaw),
    )


def rotate_vector(vector: Vector3, quaternion: Quaternion) -> Vector3:
    """Rotate a vector with a normalized x/y/z/w quaternion."""

    vx, vy, vz = _finite_tuple(vector, 3, 'vector')
    qx, qy, qz, qw = normalize_quaternion(quaternion)
    # Equivalent to q * [v, 0] * conjugate(q), without temporary quaternions.
    tx = 2.0 * (qy * vz - qz * vy)
    ty = 2.0 * (qz * vx - qx * vz)
    tz = 2.0 * (qx * vy - qy * vx)
    return (
        vx + qw * tx + qy * tz - qz * ty,
        vy + qw * ty + qz * tx - qx * tz,
        vz + qw * tz + qx * ty - qy * tx,
    )


def compose_pose_right(
    position: Iterable[object],
    orientation_xyzw: Iterable[object],
    offset_position: Iterable[object],
    offset_orientation_xyzw: Iterable[object],
) -> PoseTuple:
    """Compose ``parent<-object * object<-goal`` rigid transforms.

    ``offset_position`` is therefore expressed in the object frame, not in the
    parent/base frame.  The returned position and quaternion express the goal
    pose in the same parent frame as the first pose.
    """

    parent_position = _finite_tuple(position, 3, 'position')
    parent_orientation = normalize_quaternion(orientation_xyzw)
    local_position = _finite_tuple(offset_position, 3, 'offset_position')
    local_orientation = normalize_quaternion(offset_orientation_xyzw)
    rotated_offset = rotate_vector(local_position, parent_orientation)
    return (
        tuple(
            parent_value + offset_value
            for parent_value, offset_value in zip(
                parent_position,
                rotated_offset,
            )
        ),
        quaternion_multiply(parent_orientation, local_orientation),
    )


def observation_to_cartesian_target(
    observation: ObjectObservation,
    *,
    target_frame: str = 'base',
) -> CartesianTarget:
    """Convert a frame-verified observation to the backend's 6D target."""

    if not isinstance(observation, ObjectObservation):
        raise TypeError('observation must be an ObjectObservation')
    frame = str(target_frame).strip()
    if not frame:
        raise ValueError('target_frame must not be empty')
    if observation.frame_id != frame:
        raise ValueError(
            f'observation frame {observation.frame_id!r} does not match '
            f'target frame {frame!r}'
        )
    roll, pitch, yaw = quaternion_to_rpy_deg(observation.orientation_xyzw)
    return (
        *observation.position,
        roll,
        pitch,
        yaw,
    )


def transform_observation(
    observation: ObjectObservation,
    *,
    target_frame: str,
    translation: Iterable[object],
    rotation_xyzw: Iterable[object],
) -> ObjectObservation:
    """Apply a target-from-source rigid transform to an observation."""

    target = str(target_frame).strip()
    if not target:
        raise ValueError('target_frame must not be empty')
    offset = _finite_tuple(translation, 3, 'translation')
    rotation = normalize_quaternion(rotation_xyzw)
    rotated = rotate_vector(observation.position, rotation)
    return replace(
        observation,
        frame_id=target,
        position=tuple(a + b for a, b in zip(offset, rotated)),
        orientation_xyzw=quaternion_multiply(
            rotation,
            observation.orientation_xyzw,
        ),
    )


__all__ = [
    'CartesianTarget',
    'ObjectObservation',
    'PoseTuple',
    'Quaternion',
    'RollPitchYawDegrees',
    'Vector3',
    'compose_pose_right',
    'normalize_quaternion',
    'observation_to_cartesian_target',
    'quaternion_multiply',
    'quaternion_to_rpy_deg',
    'rotate_vector',
    'transform_observation',
]
