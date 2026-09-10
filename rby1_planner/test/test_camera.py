from types import SimpleNamespace

import pytest


pytest.importorskip('rclpy')
rby1_msgs = pytest.importorskip('rby1_msgs.msg')
pytest.importorskip('tf2_ros')

if not hasattr(rby1_msgs, 'DetectedObjectPose'):
    pytest.skip(
        'rby1_msgs/DetectedObjectPose is not generated',
        allow_module_level=True,
    )

from rclpy.clock import ClockType
from rby1_msgs.msg import DetectedObjectPose
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
    def __init__(self, *, fail=False):
        self.fail = fail
        self.calls = []
        self.clear_calls = 0

    def clear(self):
        self.clear_calls += 1

    def lookup_transform(self, target, source, stamp, timeout):
        self.calls.append((target, source, stamp.nanoseconds, timeout.nanoseconds))
        if self.fail:
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
    object_id='tag_0',
    frame_id='camera',
    stamp_ns=1_000_000_000,
    position=(1.0, 0.0, 0.0),
    orientation=(0.0, 0.0, 0.0, 1.0),
    confidence=0.9,
):
    message = DetectedObjectPose()
    message.object_id = object_id
    message.header.frame_id = frame_id
    message.header.stamp.sec = stamp_ns // 1_000_000_000
    message.header.stamp.nanosec = stamp_ns % 1_000_000_000
    (
        message.pose.position.x,
        message.pose.position.y,
        message.pose.position.z,
    ) = position
    (
        message.pose.orientation.x,
        message.pose.orientation.y,
        message.pose.orientation.z,
        message.pose.orientation.w,
    ) = orientation
    message.confidence = confidence
    return message


def test_client_caches_and_transforms_newest_valid_observation():
    node = FakeNode(1_200_000_000)
    buffer = FakeBuffer()
    client = CameraClient(node, tf_buffer=buffer, minimum_confidence=0.5)

    client._object_pose_callback(_message())
    client._object_pose_callback(_message(stamp_ns=900_000_000))
    observation = client.require_observation('tag_0')

    assert client.cached_object_ids() == ('tag_0',)
    assert observation.frame_id == 'base'
    assert observation.position == pytest.approx((0.0, 2.0, 0.0))
    assert buffer.calls[0][:3] == ('base', 'camera', 1_000_000_000)


def test_client_rejects_stale_low_confidence_and_missing_tf():
    node = FakeNode(1_100_000_000)
    client = CameraClient(
        node,
        tf_buffer=FakeBuffer(fail=True),
        minimum_confidence=0.5,
    )
    client._object_pose_callback(_message(confidence=0.4))
    assert client.get_observation('tag_0') is None
    assert 'confidence' in client.unavailable_reason('tag_0')

    client._object_pose_callback(_message(stamp_ns=1_050_000_000))
    with pytest.raises(ObservationUnavailable, match='cannot transform'):
        client.require_observation('tag_0')

    node.clock.nanoseconds = 2_000_000_000
    assert client.get_observation('tag_0') is None
    assert 'stale' in client.unavailable_reason('tag_0')


def test_capture_marker_requires_a_strictly_newer_detection():
    node = FakeNode(1_000_000_000)
    client = CameraClient(node, tf_buffer=FakeBuffer())
    marker = client.capture_marker()
    client._object_pose_callback(
        _message(frame_id='base', stamp_ns=marker)
    )
    assert client.get_observation('tag_0', newer_than_ns=marker) is None

    client._object_pose_callback(
        _message(frame_id='base', stamp_ns=marker + 1)
    )
    node.clock.nanoseconds = marker + 1
    client._object_pose_callback(
        _message(frame_id='base', stamp_ns=marker + 1)
    )
    assert client.get_observation(
        'tag_0',
        newer_than_ns=marker,
    ) is not None


def test_backward_ros_clock_jump_clears_the_old_epoch():
    node = FakeNode(10_000_000_000)
    buffer = FakeBuffer()
    client = CameraClient(node, tf_buffer=buffer)
    client._object_pose_callback(
        _message(frame_id='base', stamp_ns=10_000_000_000)
    )

    node.clock.nanoseconds = 1_000_000_000
    client._object_pose_callback(
        _message(frame_id='base', stamp_ns=1_000_000_000)
    )

    assert client.latest_raw('tag_0').stamp_ns == 1_000_000_000
    assert buffer.clear_calls == 1


def test_future_dated_sample_does_not_poison_the_object_cache():
    node = FakeNode(1_000_000_000)
    client = CameraClient(node, tf_buffer=FakeBuffer())
    client._object_pose_callback(
        _message(frame_id='base', stamp_ns=1_000_000_000)
    )

    client._object_pose_callback(
        _message(frame_id='base', stamp_ns=10_000_000_000)
    )
    assert client.latest_raw('tag_0').stamp_ns == 1_000_000_000

    node.clock.nanoseconds = 1_100_000_000
    client._object_pose_callback(
        _message(frame_id='base', stamp_ns=1_100_000_000)
    )
    assert client.latest_raw('tag_0').stamp_ns == 1_100_000_000


def test_receipt_age_limits_an_observation_with_tolerated_clock_skew():
    node = FakeNode(1_000_000_000)
    client = CameraClient(node, tf_buffer=FakeBuffer(), max_age_sec=0.5)
    client._object_pose_callback(
        _message(frame_id='base', stamp_ns=1_050_000_000)
    )
    assert client.get_observation('tag_0') is not None

    node.clock.nanoseconds = 1_510_000_000
    assert client.get_observation('tag_0') is None
    assert 'stale since receipt' in client.unavailable_reason('tag_0')


def test_capture_marker_rejects_a_cached_sample_with_positive_stamp_skew():
    node = FakeNode(1_000_000_000)
    client = CameraClient(node, tf_buffer=FakeBuffer())
    client._object_pose_callback(
        _message(frame_id='base', stamp_ns=1_040_000_000)
    )

    marker = client.capture_marker()
    assert client.get_observation('tag_0', newer_than_ns=marker) is None


def test_positive_tf_timeout_is_rejected_for_non_blocking_executor():
    node = FakeNode(1_000_000_000)
    with pytest.raises(ValueError, match='must be 0.0'):
        CameraClient(node, tf_buffer=FakeBuffer(), tf_timeout_sec=0.1)
