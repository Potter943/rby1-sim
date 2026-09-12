#!/usr/bin/env python3
"""Publish the configurable end-effector-to-AprilTag-camera static TF."""

from __future__ import annotations

import math
from typing import Iterable, Tuple

from geometry_msgs.msg import TransformStamped
import rclpy
from rclpy.node import Node
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster


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


class CameraMountTfPublisher(Node):
    """Broadcast one calibrated static transform loaded from parameters."""

    def __init__(self) -> None:
        super().__init__('camera_mount_tf_publisher')

        self.declare_parameter('parent_frame', 'ee_right')
        self.declare_parameter('camera_frame', 'camera_link')
        self.declare_parameter('translation_xyz', [0.0, 0.0, 0.0])
        self.declare_parameter('rotation_xyzw', [0.0, 0.0, 0.0, 1.0])

        parent_frame = str(
            self.get_parameter('parent_frame').value
        ).strip()
        camera_frame = str(
            self.get_parameter('camera_frame').value
        ).strip()
        translation = _finite_values(
            self.get_parameter('translation_xyz').value,
            3,
            'translation_xyz',
        )
        rotation = _finite_values(
            self.get_parameter('rotation_xyzw').value,
            4,
            'rotation_xyzw',
        )
        if not parent_frame:
            raise ValueError('parent_frame must not be empty')
        if not camera_frame:
            raise ValueError('camera_frame must not be empty')
        if parent_frame == camera_frame:
            raise ValueError('parent_frame and camera_frame must differ')

        quaternion_norm = math.sqrt(sum(value * value for value in rotation))
        if quaternion_norm <= 1.0e-12:
            raise ValueError('rotation_xyzw must be a non-zero quaternion')
        normalized_rotation = tuple(
            value / quaternion_norm
            for value in rotation
        )

        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = parent_frame
        transform.child_frame_id = camera_frame
        (
            transform.transform.translation.x,
            transform.transform.translation.y,
            transform.transform.translation.z,
        ) = translation
        (
            transform.transform.rotation.x,
            transform.transform.rotation.y,
            transform.transform.rotation.z,
            transform.transform.rotation.w,
        ) = normalized_rotation

        self.broadcaster = StaticTransformBroadcaster(self)
        self.broadcaster.sendTransform(transform)
        self.get_logger().info(
            f'Camera mount TF ready: {parent_frame} -> {camera_frame}; '
            f'translation={translation}; rotation={normalized_rotation}'
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CameraMountTfPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
