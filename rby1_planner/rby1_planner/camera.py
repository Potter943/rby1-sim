"""Adapt AprilTag detections and TF poses for planner commands."""
from __future__ import annotations

from collections import deque
from dataclasses import dataclass
import math
import threading
import time
from typing import Deque, Dict, Optional, Tuple

from apriltag_msgs.msg import AprilTagDetectionArray
from rclpy.duration import Duration
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener

from .observation import (
    CartesianTarget,
    ObjectObservation,
    observation_to_cartesian_target,
    transform_observation,
)


# Both rby1_camera launches route apriltag_ros-compatible detection arrays to
# this existing planner parameter/topic. Pose itself arrives through /tf.
OBJECT_POSE_TOPIC = 'perception/object_pose'
OBJECT_POSE_TARGET_FRAME = 'base'
OBJECT_POSE_QOS_DEPTH = 5
TF_QOS_DEPTH = 5


class ObservationUnavailable(RuntimeError):
    """Raised when an object has no observation that is safe to consume."""


@dataclass(frozen=True)
class _TagDetection:
    """Metadata needed to retrieve an AprilTag pose from TF on demand."""

    object_id: str
    source_frame: str
    tag_frames: Tuple[str, ...]
    stamp_ns: int
    received_at_ns: int
    decision_margin: float


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


def _stamp_nanoseconds(header) -> int:
    frame_id = str(header.frame_id).strip()
    if not frame_id:
        raise ValueError('header.frame_id must not be empty')
    sec = int(header.stamp.sec)
    nanosec = int(header.stamp.nanosec)
    if sec < 0 or not 0 <= nanosec < 1_000_000_000:
        raise ValueError('header.stamp is invalid')
    if sec == 0 and nanosec == 0:
        raise ValueError('header.stamp must contain capture time')
    return sec * 1_000_000_000 + nanosec


class CameraClient:
    """Convert real or simulated AprilTag output into planner coordinates.

    ``apriltag_ros`` puts detection identity in ``AprilTagDetectionArray`` and
    publishes each 3D tag pose on TF. The mock camera follows that same split.
    This client caches detection timestamps, then resolves ``target<-tag`` at
    the original image time when planner code requests a result.

    ``get_object_position()`` and ``require_object_position()`` are the direct
    command adapters. They return ``(x, y, z, roll_deg, pitch_deg, yaw_deg)``,
    exactly the six-value Cartesian pose accepted by planner Task commands.
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
        self._tag_detections: Dict[str, Deque[_TagDetection]] = {}

        # Kept as an input-free compatibility cache for existing callers that
        # directly exercise _object_pose_callback with DetectedObjectPose.
        # Production subscriptions use only the AprilTag interface above.
        self._observations: Dict[str, ObjectObservation] = {}
        self._last_unavailable: Dict[str, str] = {}
        self._last_warning_at: Dict[str, float] = {}
        self._last_clock_ns: Optional[int] = None

        self._tf_buffer = tf_buffer if tf_buffer is not None else Buffer()
        self._tf_listener = None
        if tf_buffer is None:
            # The robot can publish /tf faster than a Qt-integrated executor
            # consumes callbacks. Keep only recent transforms so a reliable
            # depth-100 queue cannot leave perception almost a second behind.
            tf_qos = QoSProfile(
                history=QoSHistoryPolicy.KEEP_LAST,
                depth=TF_QOS_DEPTH,
                reliability=QoSReliabilityPolicy.BEST_EFFORT,
                durability=QoSDurabilityPolicy.VOLATILE,
            )
            self._tf_listener = TransformListener(
                self._tf_buffer,
                node,
                spin_thread=False,
                qos=tf_qos,
            )

        qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=OBJECT_POSE_QOS_DEPTH,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
        )
        self._subscription = node.create_subscription(
            AprilTagDetectionArray,
            topic,
            self._apriltag_callback,
            qos,
        )

    def capture_marker(self) -> int:
        """Return a ROS timestamp for selecting a later streamed detection."""

        return int(self._clock.now().nanoseconds)

    def cached_object_ids(self) -> Tuple[str, ...]:
        """Return all IDs with a syntactically valid cached detection."""

        with self._lock:
            return tuple(sorted(
                set(self._tag_detections) | set(self._observations)
            ))

    def clear(self, object_id: Optional[str] = None) -> None:
        """Clear one cached object or the complete detection cache."""

        with self._lock:
            if object_id is None:
                self._tag_detections.clear()
                self._observations.clear()
                self._last_unavailable.clear()
            else:
                key = str(object_id).strip()
                self._tag_detections.pop(key, None)
                self._observations.pop(key, None)
                self._last_unavailable.pop(key, None)

    def latest_raw(self, object_id: str) -> Optional[ObjectObservation]:
        """Return a legacy directly supplied pose without validity checks.

        AprilTag detection messages intentionally contain no 3D pose, so their
        coordinates are available through ``get_observation()`` after TF has
        received the matching tag transform.
        """

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
        """Return a fresh tag pose transformed into ``target_frame``."""

        key = str(object_id).strip()
        if not key:
            raise ValueError('object_id must not be empty')
        frame = (
            self.target_frame
            if target_frame is None
            else str(target_frame).strip()
        )
        if not frame:
            raise ValueError('target_frame must not be empty')
        maximum_age = (
            self.max_age_sec
            if max_age_sec is None
            else _duration_seconds(
                max_age_sec,
                'max_age_sec',
                allow_zero=False,
            )
        )
        confidence_limit = (
            self.minimum_confidence
            if minimum_confidence is None
            else _finite_number(minimum_confidence, 'minimum_confidence')
        )
        if not 0.0 <= confidence_limit <= 1.0:
            raise ValueError('minimum_confidence must be in the range [0, 1]')
        threshold = self._newer_than_threshold(newer_than_ns)

        with self._lock:
            tag_detections = tuple(self._tag_detections.get(key, ()))
            direct_observation = self._observations.get(key)
        tag_detection = tag_detections[-1] if tag_detections else None

        # If both interfaces were fed during a transition, use the newest
        # capture and prefer the AprilTag interface for an equal timestamp.
        if tag_detection is not None and (
            direct_observation is None
            or tag_detection.stamp_ns >= direct_observation.stamp_ns
        ):
            # Detection messages and their TF can arrive on different DDS
            # callbacks. Try newest first, then a short recent history so a
            # just-arrived detection does not hide the previous resolvable TF.
            for candidate in reversed(tag_detections):
                if not self._sample_is_usable(
                    key,
                    stamp_ns=candidate.stamp_ns,
                    received_at_ns=candidate.received_at_ns,
                    confidence=1.0,
                    maximum_age=maximum_age,
                    confidence_limit=confidence_limit,
                    newer_than_ns=threshold,
                ):
                    continue
                result = self._resolve_tag_observation(candidate, frame)
                if result is not None:
                    return result
            return None

        if direct_observation is None:
            return self._unavailable(key, f'object {key!r} has not been observed')
        if not self._sample_is_usable(
            key,
            stamp_ns=direct_observation.stamp_ns,
            received_at_ns=direct_observation.received_at_ns,
            confidence=direct_observation.confidence,
            maximum_age=maximum_age,
            confidence_limit=confidence_limit,
            newer_than_ns=threshold,
        ):
            return None
        return self._transform_direct_observation(direct_observation, frame)

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

    def get_object_position(
        self,
        object_id: str,
        **kwargs,
    ) -> Optional[CartesianTarget]:
        """Return a Task-ready ``x/y/z/R/P/Y-degrees`` Cartesian position."""

        observation = self.get_observation(object_id, **kwargs)
        if observation is None:
            return None
        return observation_to_cartesian_target(
            observation,
            target_frame=observation.frame_id,
        )

    def require_object_position(
        self,
        object_id: str,
        **kwargs,
    ) -> CartesianTarget:
        """Return a Task-ready Cartesian position or raise when unavailable."""

        observation = self.require_observation(object_id, **kwargs)
        return observation_to_cartesian_target(
            observation,
            target_frame=observation.frame_id,
        )

    # Explicit Cartesian aliases make call sites self-documenting while the
    # object-position names remain concise for Task authoring.
    get_cartesian_position = get_object_position
    require_cartesian_position = require_object_position

    def unavailable_reason(self, object_id: str) -> str:
        """Return the most recent query failure for an object, if any."""

        with self._lock:
            return self._last_unavailable.get(str(object_id).strip(), '')

    def _apriltag_callback(self, message: AprilTagDetectionArray) -> None:
        """Cache tag identities; their poses are deliberately read from TF."""

        try:
            received_at_ns = int(self._clock.now().nanoseconds)
            stamp_ns = _stamp_nanoseconds(message.header)
            source_frame = str(message.header.frame_id).strip()
        except (AttributeError, TypeError, ValueError) as exc:
            self._warn_throttled(
                'invalid',
                f'Invalid AprilTag detection array ignored: {exc}',
            )
            return

        if self._prepare_clock_epoch(received_at_ns):
            self._clear_tf_after_clock_reset()
        if not self._capture_time_is_usable(stamp_ns, received_at_ns):
            return

        for detection in message.detections:
            try:
                tag_id = int(detection.id)
                if isinstance(detection.id, bool) or tag_id < 0:
                    raise ValueError('detection.id must be nonnegative')
                family = str(detection.family).strip()
                if not family:
                    raise ValueError('detection.family must not be empty')
                margin = _finite_number(
                    detection.decision_margin,
                    'detection.decision_margin',
                )
                hamming = int(detection.hamming)
                if isinstance(detection.hamming, bool) or hamming < 0:
                    raise ValueError('detection.hamming must be nonnegative')

                object_id = f'tag_{tag_id}'
                default_frame = f'{family}:{tag_id}'
                frames = [object_id, default_frame]
                if not family.startswith('tag'):
                    frames.append(f'tag{family}:{tag_id}')
                tag_frames = tuple(dict.fromkeys(frames))
                sample = _TagDetection(
                    object_id=object_id,
                    source_frame=source_frame,
                    tag_frames=tag_frames,
                    stamp_ns=stamp_ns,
                    received_at_ns=received_at_ns,
                    decision_margin=margin,
                )
            except (AttributeError, TypeError, ValueError) as exc:
                self._warn_throttled(
                    'invalid_detection',
                    f'Invalid AprilTag detection ignored: {exc}',
                )
                continue

            with self._lock:
                history = self._tag_detections.get(object_id)
                current = history[-1] if history else None
                if current is not None:
                    if sample.stamp_ns < current.stamp_ns:
                        continue
                    if (
                        sample.stamp_ns == current.stamp_ns
                        and sample.decision_margin < current.decision_margin
                    ):
                        continue
                if history is None:
                    history = deque(maxlen=OBJECT_POSE_QOS_DEPTH)
                    self._tag_detections[object_id] = history
                if current is not None and sample.stamp_ns == current.stamp_ns:
                    history[-1] = sample
                else:
                    history.append(sample)
                self._last_unavailable.pop(object_id, None)

    def _object_pose_callback(self, message) -> None:
        """Accept the former direct-pose message for source compatibility.

        No ROS subscription uses this callback. It can be removed once all
        downstream code has migrated to the AprilTag detection/TF boundary.
        """

        try:
            received_at_ns = int(self._clock.now().nanoseconds)
            stamp_ns = _stamp_nanoseconds(message.header)
            observation = ObjectObservation(
                object_id=message.object_id,
                frame_id=message.header.frame_id,
                stamp_ns=stamp_ns,
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

        if self._prepare_clock_epoch(received_at_ns):
            self._clear_tf_after_clock_reset()
        if not self._capture_time_is_usable(stamp_ns, received_at_ns):
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

    def _resolve_tag_observation(
        self,
        detection: _TagDetection,
        target_frame: str,
    ) -> Optional[ObjectObservation]:
        stamp = Time(
            nanoseconds=detection.stamp_ns,
            clock_type=self._clock.clock_type,
        )
        failures = []
        for tag_frame in detection.tag_frames:
            try:
                transform = self._tf_buffer.lookup_transform(
                    target_frame,
                    tag_frame,
                    stamp,
                    timeout=Duration(seconds=self.tf_timeout_sec),
                )
                result = ObjectObservation(
                    object_id=detection.object_id,
                    frame_id=target_frame,
                    stamp_ns=detection.stamp_ns,
                    received_at_ns=detection.received_at_ns,
                    position=(
                        transform.transform.translation.x,
                        transform.transform.translation.y,
                        transform.transform.translation.z,
                    ),
                    orientation_xyzw=(
                        transform.transform.rotation.x,
                        transform.transform.rotation.y,
                        transform.transform.rotation.z,
                        transform.transform.rotation.w,
                    ),
                    # AprilTag decision_margin is not a normalized confidence.
                    # A present detection has already passed apriltag_ros's
                    # configured hamming filter, so expose validity as 1.0.
                    confidence=1.0,
                )
            except (AttributeError, TransformException, ValueError) as exc:
                failures.append(f'{tag_frame!r}: {exc}')
                continue
            self._clear_unavailable(detection.object_id)
            return result

        return self._unavailable(
            detection.object_id,
            f'cannot transform object {detection.object_id!r} to '
            f'{target_frame!r}; tried {", ".join(failures)}',
        )

    def _transform_direct_observation(
        self,
        observation: ObjectObservation,
        target_frame: str,
    ) -> Optional[ObjectObservation]:
        if observation.frame_id == target_frame:
            self._clear_unavailable(observation.object_id)
            return observation

        try:
            stamp = Time(
                nanoseconds=observation.stamp_ns,
                clock_type=self._clock.clock_type,
            )
            transform = self._tf_buffer.lookup_transform(
                target_frame,
                observation.frame_id,
                stamp,
                timeout=Duration(seconds=self.tf_timeout_sec),
            )
            result = transform_observation(
                observation,
                target_frame=target_frame,
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
        except (AttributeError, TransformException, ValueError) as exc:
            return self._unavailable(
                observation.object_id,
                f'cannot transform object {observation.object_id!r} from '
                f'{observation.frame_id!r} to {target_frame!r}: {exc}',
            )
        self._clear_unavailable(observation.object_id)
        return result

    def _sample_is_usable(
        self,
        key: str,
        *,
        stamp_ns: int,
        received_at_ns: int,
        confidence: float,
        maximum_age: float,
        confidence_limit: float,
        newer_than_ns: Optional[int],
    ) -> bool:
        now_ns = int(self._clock.now().nanoseconds)
        age = (now_ns - stamp_ns) / 1_000_000_000.0
        if age < -self.future_tolerance_sec:
            self._unavailable(
                key,
                f'object {key!r} observation is future-dated by {-age:.3f}s',
            )
            return False
        if age > maximum_age:
            self._unavailable(
                key,
                f'object {key!r} observation is stale ({age:.3f}s)',
            )
            return False

        receipt_age = (now_ns - received_at_ns) / 1_000_000_000.0
        if receipt_age < -self.future_tolerance_sec:
            self._unavailable(
                key,
                f'object {key!r} receipt time is future-dated by '
                f'{-receipt_age:.3f}s',
            )
            return False
        if receipt_age > maximum_age:
            self._unavailable(
                key,
                f'object {key!r} observation is stale since receipt '
                f'({receipt_age:.3f}s)',
            )
            return False
        if confidence < confidence_limit:
            self._unavailable(
                key,
                f'object {key!r} confidence {confidence:.3f} is below '
                f'{confidence_limit:.3f}',
            )
            return False
        if newer_than_ns is not None and (
            stamp_ns <= newer_than_ns or received_at_ns <= newer_than_ns
        ):
            self._unavailable(
                key,
                f'object {key!r} has no observation after capture marker',
            )
            return False
        return True

    @staticmethod
    def _newer_than_threshold(value: Optional[int]) -> Optional[int]:
        if value is None:
            return None
        if isinstance(value, bool) or int(value) < 0:
            raise ValueError('newer_than_ns must be nonnegative')
        return int(value)

    def _prepare_clock_epoch(self, received_at_ns: int) -> bool:
        tolerance_ns = int(self.future_tolerance_sec * 1_000_000_000)
        rolled_back = False
        with self._lock:
            if (
                self._last_clock_ns is not None
                and received_at_ns + tolerance_ns < self._last_clock_ns
            ):
                self._tag_detections.clear()
                self._observations.clear()
                self._last_unavailable.clear()
                rolled_back = True
            self._last_clock_ns = received_at_ns
        return rolled_back

    def _capture_time_is_usable(
        self,
        stamp_ns: int,
        received_at_ns: int,
    ) -> bool:
        tolerance_ns = int(self.future_tolerance_sec * 1_000_000_000)
        future_offset_ns = stamp_ns - received_at_ns
        if future_offset_ns <= tolerance_ns:
            return True
        self._warn_throttled(
            'future',
            'Future-dated object observation ignored: '
            f'{future_offset_ns / 1_000_000_000:.3f}s ahead of ROS time',
        )
        return False

    def _clear_tf_after_clock_reset(self) -> None:
        clear_tf = getattr(self._tf_buffer, 'clear', None)
        if not callable(clear_tf):
            return
        try:
            clear_tf()
        except Exception as exc:
            self._warn_throttled(
                'tf_clear',
                f'Could not clear TF after ROS clock reset: {exc}',
            )

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
    'TF_QOS_DEPTH',
]
