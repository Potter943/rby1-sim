"""Executable entry point for the robot-facing control backend node."""
from __future__ import annotations

import time

import rclpy

from .topic_backend import TopicControlBackendNode


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TopicControlBackendNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.shutdown_safely(turn_stream_off=True)
            deadline = time.monotonic() + 1.0
            while rclpy.ok() and time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=0.05)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
