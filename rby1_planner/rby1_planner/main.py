"""Executable entry point for the idle-by-default planner frontend."""
from __future__ import annotations

import time

import rclpy

from .planner_node import PlannerNode


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PlannerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        # Give reliable cancel/stop publications a short executor window
        # before destroying their publishers. This also covers commands sent
        # directly through the inherited frontend API rather than TaskRunner.
        deadline = time.monotonic() + 0.5
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
