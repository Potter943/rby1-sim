"""Validated object-pose subscriber and TF client for planner code."""
from __future__ import annotations

import math
import threading
import time
from typing import Dict, Optional, Tuple

from rclpy.duration import Duration
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)
from rclpy.time import Time
from rby1_msgs.msg import DetectedObjectPose
from tf2_ros import Buffer, TransformException, TransformListener

from .observation import ObjectObservation, transform_observation


OBJECT_POSE_TOPIC = 'perception/object_pose'
OBJECT_POSE_TARGET_FRAME = 'base'
OBJECT_POSE_QOS_DEPTH = 5


class ObservationUnavailable(RuntimeError):
    """Raised when an object has no observation that is safe to consume."""


def _finite_number(value: object, label: str) -> float:
    if isinstance(value, bool):
        raise ValueError(f'{label} must be a finite number')
    try:
        result = float(value)
    except (TypeError, ValueError) as exc:
        raise ValueError(f'{label} must be a finite number') from exc
    if not math.isfinite(result):
        raise ValueError(f'{label} must be a finite number')
    return result


def _duration_seconds(value: object, label: str, *, allow_zero: bool) -> float:
    result = _finite_number(value, label)
    if result < 0.0 or (result == 0.0 and not allow_zero):
        qualifier = 'nonnegative' if allow_zero else 'positive'
        raise ValueError(f'{label} must be a {qualifier} finite number')
    return result


class CameraClient:
    """Cache typed detections and expose fresh poses in a requested frame.

    The camera and detector are streaming producers. ``capture_marker()`` does
    not trigger camera hardware; it returns a ROS timestamp that callers pass
    as ``newer_than_ns`` when they need an observation from a subsequent image.
    """

    def __init__(
        self,
        node,
        *,
        object_pose_topic: str = OBJECT_POSE_TOPIC,
        target_frame: str = OBJECT_POSE_TARGET_FRAME,
        max_age_sec: float = 0.5,
        minimum_confidence: float = 0.0,
        future_tolerance_sec: float = 0.05,
        tf_timeout_sec: float = 0.0,
        tf_buffer: Optional[Buffer] = None,
    ) -> None:
        topic = str(object_pose_topic).strip()
        target = str(target_frame).strip()
        if not topic:
            raise ValueError('object_pose_topic must not be empty')
        if not target:
            raise ValueError('target_frame must not be empty')
        self.max_age_sec = _duration_seconds(
            max_age_sec,
            'max_age_sec',
            allow_zero=False,
        )
        self.minimum_confidence = _finite_number(
            minimum_confidence,
            'minimum_confidence',
        )
        if not 0.0 <= self.minimum_confidence <= 1.0:
            raise ValueError('minimum_confidence must be in the range [0, 1]')
        self.future_tolerance_sec = _duration_seconds(
            future_tolerance_sec,
            'future_tolerance_sec',
            allow_zero=True,
        )
        self.tf_timeout_sec = _duration_seconds(
            tf_timeout_sec,
            'tf_timeout_sec',
            allow_zero=True,
        )
        if self.tf_timeout_sec != 0.0:
            raise ValueError(
                'tf_timeout_sec must be 0.0 because CameraClient uses a '
                'non-blocking TF lookup in the planner executor'
            )

        self._node = node
        self._clock = node.get_clock()
        self.object_pose_topic = topic
        self.target_frame = target
        self._lock = threading.RLock()
        self._observations: Dict[str, ObjectObservation] = {}
        self._last_unavailable: Dict[str, str] = {}
        self._last_warning_at: Dict[str, float] = {}
        self._last_clock_ns: Optional[int] = None

        self._tf_buffer = tf_buffer if tf_buffer is not None else Buffer()
        self._tf_listener = None
        if tf_buffer is None:
            self._tf_listener = TransformListener(
                self._tf_buffer,
                node,
                spin_thread=False,
            )

        qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=OBJECT_POSE_QOS_DEPTH,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
        )
        self._subscription = node.create_subscription(
            DetectedObjectPose,
            topic,
            self._object_pose_callback,
            qos,
        )

    def capture_marker(self) -> int:
        """Return a ROS timestamp for selecting a later streamed detection."""

        return int(self._clock.now().nanoseconds)

    def cached_object_ids(self) -> Tuple[str, ...]:
        """Return all IDs with a syntactically valid cached observation."""

        with self._lock:
            return tuple(sorted(self._observations))

    def clear(self, object_id: Optional[str] = None) -> None:
        """Clear one cached object or the complete observation cache."""

        with self._lock:
            if object_id is None:
                self._observations.clear()
                self._last_unavailable.clear()
            else:
                key = str(object_id).strip()
                self._observations.pop(key, None)
                self._last_unavailable.pop(key, None)

    def latest_raw(self, object_id: str) -> Optional[ObjectObservation]:
        """Return the newest cached source-frame value without freshness checks."""

        key = str(object_id).strip()
        with self._lock:
            return self._observations.get(key)

    def get_observation(
        self,
        object_id: str,
        *,
        target_frame: Optional[str] = None,
        max_age_sec: Optional[float] = None,
        minimum_confidence: Optional[float] = None,
        newer_than_ns: Optional[int] = None,
    ) -> Optional[ObjectObservation]:
        """Return a fresh observation transformed into ``target_frame``.

        Invalid, stale, low-confidence, future-dated, or currently
        untransformable observations return ``None``. ``require_observation``
        exposes the corresponding reason as an exception.
        """

        key = str(object_id).strip()
        if not key:
            raise ValueError('object_id must not be empty')
        frame = self.target_frame if target_frame is None else str(target_frame).strip()
        if not frame:
            raise ValueError('target_frame must not be empty')
        maximum_age = self.max_age_sec if max_age_sec is None else _duration_seconds(
            max_age_sec,
            'max_age_sec',
            allow_zero=False,
        )
        confidence_limit = (
            self.minimum_confidence
            if minimum_confidence is None
            else _finite_number(minimum_confidence, 'minimum_confidence')
        )
        if not 0.0 <= confidence_limit <= 1.0:
            raise ValueError('minimum_confidence must be in the range [0, 1]')

        with self._lock:
            observation = self._observations.get(key)
        if observation is None:
            return self._unavailable(key, f'object {key!r} has not been observed')

        now_ns = int(self._clock.now().nanoseconds)
        age = observation.age_seconds(now_ns)
        if age < -self.future_tolerance_sec:
            return self._unavailable(
                key,
                f'object {key!r} observation is future-dated by {-age:.3f}s',
            )
        if age > maximum_age:
            return self._unavailable(
                key,
                f'object {key!r} observation is stale ({age:.3f}s)',
            )
        receipt_age = (
            now_ns - observation.received_at_ns
        ) / 1_000_000_000.0
        if receipt_age < -self.future_tolerance_sec:
            return self._unavailable(
                key,
                f'object {key!r} receipt time is future-dated by '
                f'{-receipt_age:.3f}s',
            )
        if receipt_age > maximum_age:
            return self._unavailable(
                key,
                f'object {key!r} observation is stale since receipt '
                f'({receipt_age:.3f}s)',
            )
        if observation.confidence < confidence_limit:
            return self._unavailable(
                key,
                f'object {key!r} confidence {observation.confidence:.3f} is below '
                f'{confidence_limit:.3f}',
            )
        if newer_than_ns is not None:
            if isinstance(newer_than_ns, bool) or int(newer_than_ns) < 0:
                raise ValueError('newer_than_ns must be nonnegative')
            threshold = int(newer_than_ns)
            if (
                observation.stamp_ns <= threshold
                or observation.received_at_ns <= threshold
            ):
                return self._unavailable(
                    key,
                    f'object {key!r} has no observation after capture marker',
                )

        if observation.frame_id == frame:
            self._clear_unavailable(key)
            return observation

        try:
            stamp = Time(
                nanoseconds=observation.stamp_ns,
                clock_type=self._clock.clock_type,
            )
            transform = self._tf_buffer.lookup_transform(
                frame,
                observation.frame_id,
                stamp,
                timeout=Duration(seconds=self.tf_timeout_sec),
            )
            result = transform_observation(
                observation,
                target_frame=frame,
                translation=(
                    transform.transform.translation.x,
                    transform.transform.translation.y,
                    transform.transform.translation.z,
                ),
                rotation_xyzw=(
                    transform.transform.rotation.x,
                    transform.transform.rotation.y,
                    transform.transform.rotation.z,
                    transform.transform.rotation.w,
                ),
            )
        except (TransformException, ValueError) as exc:
            return self._unavailable(
                key,
                f'cannot transform object {key!r} from '
                f'{observation.frame_id!r} to {frame!r}: {exc}',
            )
        self._clear_unavailable(key)
        return result

    def require_observation(self, object_id: str, **kwargs) -> ObjectObservation:
        """Return a usable observation or raise ``ObservationUnavailable``."""

        result = self.get_observation(object_id, **kwargs)
        if result is not None:
            return result
        key = str(object_id).strip()
        with self._lock:
            detail = self._last_unavailable.get(
                key,
                f'object {key!r} observation is unavailable',
            )
        raise ObservationUnavailable(detail)

    def unavailable_reason(self, object_id: str) -> str:
        """Return the most recent query failure for an object, if any."""

        with self._lock:
            return self._last_unavailable.get(str(object_id).strip(), '')

    def _object_pose_callback(self, message: DetectedObjectPose) -> None:
        try:
            received_at_ns = int(self._clock.now().nanoseconds)
            stamp = message.header.stamp
            sec = int(stamp.sec)
            nanosec = int(stamp.nanosec)
            if sec < 0 or not 0 <= nanosec < 1_000_000_000:
                raise ValueError('header.stamp is invalid')
            if sec == 0 and nanosec == 0:
                raise ValueError('header.stamp must contain capture time')
            observation = ObjectObservation(
                object_id=message.object_id,
                frame_id=message.header.frame_id,
                stamp_ns=sec * 1_000_000_000 + nanosec,
                received_at_ns=received_at_ns,
                position=(
                    message.pose.position.x,
                    message.pose.position.y,
                    message.pose.position.z,
                ),
                orientation_xyzw=(
                    message.pose.orientation.x,
                    message.pose.orientation.y,
                    message.pose.orientation.z,
                    message.pose.orientation.w,
                ),
                confidence=message.confidence,
            )
        except (AttributeError, TypeError, ValueError) as exc:
            self._warn_throttled(
                'invalid',
                f'Invalid object observation ignored: {exc}',
            )
            return

        tolerance_ns = int(self.future_tolerance_sec * 1_000_000_000)
        clock_rolled_back = False
        with self._lock:
            # A simulation clock can jump backwards on reset. Clear both the
            # pose cache and TF history from the previous clock epoch before
            # considering any new sample.
            if (
                self._last_clock_ns is not None
                and received_at_ns + tolerance_ns < self._last_clock_ns
            ):
                self._observations.clear()
                self._last_unavailable.clear()
                clock_rolled_back = True
            self._last_clock_ns = received_at_ns

        if clock_rolled_back:
            clear_tf = getattr(self._tf_buffer, 'clear', None)
            if callable(clear_tf):
                try:
                    clear_tf()
                except Exception as exc:
                    self._warn_throttled(
                        'tf_clear',
                        f'Could not clear TF after ROS clock reset: {exc}',
                    )

        future_offset_ns = observation.stamp_ns - received_at_ns
        if future_offset_ns > tolerance_ns:
            self._warn_throttled(
                'future',
                'Future-dated object observation ignored: '
                f'{future_offset_ns / 1_000_000_000:.3f}s ahead of ROS time',
            )
            return

        with self._lock:
            current = self._observations.get(observation.object_id)
            if current is not None:
                if observation.stamp_ns < current.stamp_ns:
                    return
                if (
                    observation.stamp_ns == current.stamp_ns
                    and observation.confidence < current.confidence
                ):
                    return
            self._observations[observation.object_id] = observation
            self._last_unavailable.pop(observation.object_id, None)

    def _unavailable(self, key: str, detail: str):
        with self._lock:
            self._last_unavailable[key] = detail
        return None

    def _clear_unavailable(self, key: str) -> None:
        with self._lock:
            self._last_unavailable.pop(key, None)

    def _warn_throttled(self, key: str, message: str) -> None:
        now = time.monotonic()
        with self._lock:
            previous = self._last_warning_at.get(key)
            if previous is not None and now - previous < 2.0:
                return
            self._last_warning_at[key] = now
        self._node.get_logger().warning(message)


__all__ = [
    'CameraClient',
    'OBJECT_POSE_QOS_DEPTH',
    'OBJECT_POSE_TARGET_FRAME',
    'OBJECT_POSE_TOPIC',
    'ObservationUnavailable',
    'ObjectObservation',
]
