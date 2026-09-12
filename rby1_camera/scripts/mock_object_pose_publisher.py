#!/usr/bin/env python3
"""Publish the same detection and TF interfaces as ``apriltag_ros``."""

from __future__ import annotations

import math
import time
from typing import Iterable, Tuple

from apriltag_msgs.msg import AprilTagDetection, AprilTagDetectionArray
from geometry_msgs.msg import TransformStamped
import rclpy
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import (
    Buffer,
    TransformBroadcaster,
    TransformException,
    TransformListener,
)


def _finite_values(
    values: Iterable[object],
    count: int,
    label: str,
) -> Tuple[float, ...]:
    try:
        result = tuple(float(value) for value in values)
    except (TypeError, ValueError) as exc:
        raise ValueError(f'{label} must contain {count} finite numbers') from exc
    if len(result) != count or not all(math.isfinite(value) for value in result):
        raise ValueError(f'{label} must contain {count} finite numbers')
    return result


def _normalize_quaternion(values: Iterable[object]) -> Tuple[float, ...]:
    quaternion = _finite_values(values, 4, 'quaternion')
    norm = math.sqrt(sum(value * value for value in quaternion))
    if norm <= 1.0e-12:
        raise ValueError('quaternion must be non-zero')
    return tuple(value / norm for value in quaternion)


def _quaternion_multiply(left, right) -> Tuple[float, ...]:
    lx, ly, lz, lw = _normalize_quaternion(left)
    rx, ry, rz, rw = _normalize_quaternion(right)
    return _normalize_quaternion((
        lw * rx + lx * rw + ly * rz - lz * ry,
        lw * ry - lx * rz + ly * rw + lz * rx,
        lw * rz + lx * ry - ly * rx + lz * rw,
        lw * rw - lx * rx - ly * ry - lz * rz,
    ))


def _rotate_vector(vector, quaternion) -> Tuple[float, ...]:
    vx, vy, vz = _finite_values(vector, 3, 'vector')
    qx, qy, qz, qw = _normalize_quaternion(quaternion)
    tx = 2.0 * (qy * vz - qz * vy)
    ty = 2.0 * (qz * vx - qx * vz)
    tz = 2.0 * (qx * vy - qy * vx)
    return (
        vx + qw * tx + qy * tz - qz * ty,
        vy + qw * ty + qz * tx - qx * tz,
        vz + qw * tz + qx * ty - qy * tx,
    )


class MockObjectPosePublisher(Node):
    """Publish one fixed AprilTag detection and its timestamp-matched TF."""

    def __init__(self) -> None:
        super().__init__('mock_object_pose_publisher')

        self.declare_parameter('detections_topic', 'perception/object_pose')
        self.declare_parameter('world_frame', 'base')
        self.declare_parameter('camera_frame', 'camera_link')
        self.declare_parameter('tag_frame', 'tag_0')
        self.declare_parameter('tag_family', 'tag36h11')
        self.declare_parameter('tag_id', 0)
        # Keep the synthetic object at an explicit world pose.  This makes its
        # location independent of the robot posture when this node starts.
        self.declare_parameter(
            'object_position_in_world_xyz',
            [0.373999, -0.243073, 0.720902],
        )
        self.declare_parameter(
            'object_orientation_in_world_xyzw',
            [0.0, 0.0, 0.0, 1.0],
        )
        self.declare_parameter('decision_margin', 100.0)
        self.declare_parameter('publish_rate_hz', 10.0)
        self.declare_parameter('enabled', True)

        topic = str(self.get_parameter('detections_topic').value).strip()
        self.world_frame = str(
            self.get_parameter('world_frame').value
        ).strip()
        self.camera_frame = str(
            self.get_parameter('camera_frame').value
        ).strip()
        self.tag_frame = str(self.get_parameter('tag_frame').value).strip()
        self.tag_family = str(
            self.get_parameter('tag_family').value
        ).strip()
        self.tag_id = int(self.get_parameter('tag_id').value)
        self.fixed_object_position_in_world = _finite_values(
            self.get_parameter('object_position_in_world_xyz').value,
            3,
            'object_position_in_world_xyz',
        )
        self.object_orientation_in_world = _normalize_quaternion(
            self.get_parameter('object_orientation_in_world_xyzw').value,
        )
        self.decision_margin = float(
            self.get_parameter('decision_margin').value
        )
        rate_hz = float(self.get_parameter('publish_rate_hz').value)
        if not topic:
            raise ValueError('detections_topic must not be empty')
        if not self.world_frame:
            raise ValueError('world_frame must not be empty')
        if not self.camera_frame:
            raise ValueError('camera_frame must not be empty')
        if self.world_frame == self.camera_frame:
            raise ValueError('world_frame and camera_frame must differ')
        if not self.tag_frame:
            raise ValueError('tag_frame must not be empty')
        if not self.tag_family:
            raise ValueError('tag_family must not be empty')
        if self.tag_id < 0:
            raise ValueError('tag_id must be nonnegative')
        if not math.isfinite(self.decision_margin):
            raise ValueError('decision_margin must be finite')
        if not math.isfinite(rate_hz) or rate_hz <= 0.0:
            raise ValueError('publish_rate_hz must be a positive finite number')

        # apriltag_ros publishes its detection array with reliable depth-1 QoS.
        self.detections_pub = self.create_publisher(
            AprilTagDetectionArray,
            topic,
            1,
        )
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(
            self.tf_buffer,
            self,
            spin_thread=False,
        )
        self.tf_broadcaster = TransformBroadcaster(self)
        self.last_tf_warning_at = None
        self.publish_timer = self.create_timer(
            1.0 / rate_hz,
            self._publish_detection,
        )

        self.get_logger().info(
            'Mock AprilTag ready: '
            f'{self.resolve_topic_name(topic)}; '
            f'object fixed in {self.world_frame!r} at '
            f'{self.fixed_object_position_in_world}'
        )

    def _publish_detection(self) -> None:
        if not bool(self.get_parameter('enabled').value):
            return

        pose = self._camera_to_fixed_object_pose()
        if pose is None:
            return
        stamp, position, orientation = pose

        detection = AprilTagDetection()
        detection.family = self.tag_family
        detection.id = self.tag_id
        detection.hamming = 0
        detection.decision_margin = self.decision_margin

        detections = AprilTagDetectionArray()
        detections.header.stamp = stamp
        detections.header.frame_id = self.camera_frame
        detections.detections = [detection]
        transform = TransformStamped()
        transform.header.stamp = stamp
        transform.header.frame_id = self.camera_frame
        transform.child_frame_id = self.tag_frame
        (
            transform.transform.translation.x,
            transform.transform.translation.y,
            transform.transform.translation.z,
        ) = position
        (
            transform.transform.rotation.x,
            transform.transform.rotation.y,
            transform.transform.rotation.z,
            transform.transform.rotation.w,
        ) = orientation
        self.tf_broadcaster.sendTransform(transform)

        # Publish the TF first so consumers normally have the timestamped
        # transform by the time they process the matching detection metadata.
        self.detections_pub.publish(detections)

    def _camera_to_fixed_object_pose(self):
        try:
            world_from_camera = self.tf_buffer.lookup_transform(
                self.world_frame,
                self.camera_frame,
                Time(),
                timeout=Duration(seconds=0.0),
            )
            translation = world_from_camera.transform.translation
            rotation = world_from_camera.transform.rotation
            world_camera_position = (
                translation.x,
                translation.y,
                translation.z,
            )
            world_camera_orientation = _normalize_quaternion((
                rotation.x,
                rotation.y,
                rotation.z,
                rotation.w,
            ))
        except (AttributeError, TransformException, ValueError) as exc:
            self._warn_tf_throttled(
                f'Waiting for {self.world_frame!r} <- '
                f'{self.camera_frame!r} TF: {exc}'
            )
            return None

        # camera<-object = inverse(world<-camera) * world<-object
        camera_from_world_orientation = (
            -world_camera_orientation[0],
            -world_camera_orientation[1],
            -world_camera_orientation[2],
            world_camera_orientation[3],
        )
        world_camera_to_object = tuple(
            object_value - camera_value
            for object_value, camera_value in zip(
                self.fixed_object_position_in_world,
                world_camera_position,
            )
        )
        camera_object_position = _rotate_vector(
            world_camera_to_object,
            camera_from_world_orientation,
        )
        camera_object_orientation = _quaternion_multiply(
            camera_from_world_orientation,
            self.object_orientation_in_world,
        )

        stamp = world_from_camera.header.stamp
        if int(stamp.sec) == 0 and int(stamp.nanosec) == 0:
            stamp = self.get_clock().now().to_msg()
        return stamp, camera_object_position, camera_object_orientation

    def _warn_tf_throttled(self, message: str) -> None:
        now = time.monotonic()
        if (
            self.last_tf_warning_at is not None
            and now - self.last_tf_warning_at < 2.0
        ):
            return
        self.last_tf_warning_at = now
        self.get_logger().warning(message)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MockObjectPosePublisher()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
