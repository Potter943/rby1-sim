from types import SimpleNamespace

import pytest


pytest.importorskip('rclpy')
apriltag_msgs = pytest.importorskip('apriltag_msgs.msg')
pytest.importorskip('tf2_ros')

if not hasattr(apriltag_msgs, 'AprilTagDetectionArray'):
    pytest.skip(
        'apriltag_msgs messages are not generated',
        allow_module_level=True,
    )

from apriltag_msgs.msg import AprilTagDetection, AprilTagDetectionArray
from rclpy.clock import ClockType
from tf2_ros import TransformException

from rby1_planner.camera import CameraClient, ObservationUnavailable


class FakeNow:
    def __init__(self, nanoseconds):
        self.nanoseconds = nanoseconds


class FakeClock:
    clock_type = ClockType.ROS_TIME

    def __init__(self, nanoseconds):
        self.nanoseconds = nanoseconds

    def now(self):
        return FakeNow(self.nanoseconds)


class FakeLogger:
    def warning(self, _message):
        pass


class FakeNode:
    def __init__(self, nanoseconds):
        self.clock = FakeClock(nanoseconds)

    def get_clock(self):
        return self.clock

    def get_logger(self):
        return FakeLogger()

    def create_subscription(self, message_type, topic, callback, qos):
        self.subscription = (message_type, topic, callback, qos)
        return self.subscription


class FakeBuffer:
    def __init__(self, *, fail=False, fail_stamps=()):
        self.fail = fail
        self.fail_stamps = set(fail_stamps)
        self.calls = []
        self.clear_calls = 0

    def clear(self):
        self.clear_calls += 1

    def lookup_transform(self, target, source, stamp, timeout):
        self.calls.append((target, source, stamp.nanoseconds, timeout.nanoseconds))
        if self.fail or stamp.nanoseconds in self.fail_stamps:
            raise TransformException('missing transform')
        return SimpleNamespace(
            transform=SimpleNamespace(
                translation=SimpleNamespace(x=0.0, y=1.0, z=0.0),
                rotation=SimpleNamespace(
                    x=0.0,
                    y=0.0,
                    z=2 ** -0.5,
                    w=2 ** -0.5,
                ),
            )
        )


def _message(
    *,
    tag_id=0,
    family='tag36h11',
    frame_id='camera',
    stamp_ns=1_000_000_000,
    decision_margin=100.0,
):
    message = AprilTagDetectionArray()
    message.header.frame_id = frame_id
    message.header.stamp.sec = stamp_ns // 1_000_000_000
    message.header.stamp.nanosec = stamp_ns % 1_000_000_000
    detection = AprilTagDetection()
    detection.family = family
    detection.id = tag_id
    detection.hamming = 0
    detection.decision_margin = decision_margin
    message.detections.append(detection)
    return message


def test_client_caches_and_transforms_newest_valid_observation():
    node = FakeNode(1_200_000_000)
    buffer = FakeBuffer()
    client = CameraClient(node, tf_buffer=buffer, minimum_confidence=0.5)

    client._apriltag_callback(_message())
    client._apriltag_callback(_message(stamp_ns=900_000_000))
    observation = client.require_observation('tag_0')

    assert client.cached_object_ids() == ('tag_0',)
    assert observation.frame_id == 'base'
    assert observation.position == pytest.approx((0.0, 1.0, 0.0))
    assert buffer.calls[0][:3] == ('base', 'tag_0', 1_000_000_000)


def test_client_rejects_stale_detection_and_retries_missing_tf():
    node = FakeNode(1_100_000_000)
    buffer = FakeBuffer(fail=True)
    client = CameraClient(
        node,
        tf_buffer=buffer,
        minimum_confidence=0.5,
    )
    client._apriltag_callback(_message(stamp_ns=1_050_000_000))
    with pytest.raises(ObservationUnavailable, match='cannot transform'):
        client.require_observation('tag_0')

    buffer.fail = False
    assert client.require_observation('tag_0').position == pytest.approx(
        (0.0, 1.0, 0.0)
    )

    node.clock.nanoseconds = 2_000_000_000
    assert client.get_observation('tag_0') is None
    assert 'stale' in client.unavailable_reason('tag_0')


def test_client_uses_recent_detection_when_newest_tf_has_not_arrived():
    node = FakeNode(1_300_000_000)
    buffer = FakeBuffer(fail_stamps={1_200_000_000})
    client = CameraClient(node, tf_buffer=buffer)

    client._apriltag_callback(_message(stamp_ns=1_100_000_000))
    client._apriltag_callback(_message(stamp_ns=1_200_000_000))

    observation = client.require_observation(
        'tag_0',
        newer_than_ns=1_000_000_000,
    )

    assert observation.stamp_ns == 1_100_000_000
    assert [call[2] for call in buffer.calls] == [
        1_200_000_000,
        1_200_000_000,
        1_100_000_000,
    ]


def test_capture_marker_requires_a_strictly_newer_detection():
    node = FakeNode(1_000_000_000)
    client = CameraClient(node, tf_buffer=FakeBuffer())
    marker = client.capture_marker()
    client._apriltag_callback(_message(stamp_ns=marker))
    assert client.get_observation('tag_0', newer_than_ns=marker) is None

    node.clock.nanoseconds = marker + 1
    client._apriltag_callback(_message(stamp_ns=marker + 1))
    assert client.get_observation(
        'tag_0',
        newer_than_ns=marker,
    ) is not None


def test_backward_ros_clock_jump_clears_the_old_epoch():
    node = FakeNode(10_000_000_000)
    buffer = FakeBuffer()
    client = CameraClient(node, tf_buffer=buffer)
    client._apriltag_callback(_message(stamp_ns=10_000_000_000))

    node.clock.nanoseconds = 1_000_000_000
    client._apriltag_callback(_message(stamp_ns=1_000_000_000))

    observation = client.require_observation('tag_0')
    assert observation.stamp_ns == 1_000_000_000
    assert buffer.clear_calls == 1


def test_future_dated_sample_does_not_poison_the_object_cache():
    node = FakeNode(1_000_000_000)
    buffer = FakeBuffer()
    client = CameraClient(node, tf_buffer=buffer)
    client._apriltag_callback(_message(stamp_ns=1_000_000_000))

    client._apriltag_callback(_message(stamp_ns=10_000_000_000))
    assert client.require_observation('tag_0').stamp_ns == 1_000_000_000

    node.clock.nanoseconds = 1_100_000_000
    client._apriltag_callback(_message(stamp_ns=1_100_000_000))
    assert client.require_observation('tag_0').stamp_ns == 1_100_000_000


def test_receipt_age_limits_an_observation_with_tolerated_clock_skew():
    node = FakeNode(1_000_000_000)
    client = CameraClient(node, tf_buffer=FakeBuffer(), max_age_sec=0.5)
    client._apriltag_callback(_message(stamp_ns=1_050_000_000))
    assert client.get_observation('tag_0') is not None

    node.clock.nanoseconds = 1_510_000_000
    assert client.get_observation('tag_0') is None
    assert 'stale since receipt' in client.unavailable_reason('tag_0')


def test_capture_marker_rejects_a_cached_sample_with_positive_stamp_skew():
    node = FakeNode(1_000_000_000)
    client = CameraClient(node, tf_buffer=FakeBuffer())
    client._apriltag_callback(_message(stamp_ns=1_040_000_000))

    marker = client.capture_marker()
    assert client.get_observation('tag_0', newer_than_ns=marker) is None


def test_object_position_matches_cartesian_linear_absolute_units():
    node = FakeNode(1_100_000_000)
    client = CameraClient(node, tf_buffer=FakeBuffer())
    client._apriltag_callback(_message(stamp_ns=1_050_000_000))

    position = client.require_object_position('tag_0')

    assert position == pytest.approx((0.0, 1.0, 0.0, 0.0, 0.0, 90.0))


def test_positive_tf_timeout_is_rejected_for_non_blocking_executor():
    node = FakeNode(1_000_000_000)
    with pytest.raises(ValueError, match='must be 0.0'):
        CameraClient(node, tf_buffer=FakeBuffer(), tf_timeout_sec=0.1)
