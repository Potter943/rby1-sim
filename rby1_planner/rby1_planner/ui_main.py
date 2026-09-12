"""Application entry point for the camera-aware planner UI."""
from __future__ import annotations

import signal
import sys
import time

from rby1_control.qt_compat import (
    QApplication,
    QT_BINDING,
    QTimer,
    app_exec,
)

from .planner_node import PlannerNode
from .planner_ui import PlannerMainWindow


ROS_CALLBACKS_PER_QT_TICK = 8


def _run_ros(argv) -> int:
    import rclpy

    rclpy.init(args=argv[1:])
    app = QApplication(argv)
    app.setApplicationName('RB-Y1 Planner UI')

    node = PlannerNode()
    window = PlannerMainWindow(node)
    app.installEventFilter(window)

    spin_timer = QTimer()
    spin_timer.setInterval(10)

    def drain_ros_callbacks() -> None:
        # /tf can exceed the Qt timer's 100 Hz rate. Drain a bounded batch so
        # transforms stay current without allowing ROS to starve the GUI.
        for _ in range(ROS_CALLBACKS_PER_QT_TICK):
            rclpy.spin_once(node, timeout_sec=0.0)

    spin_timer.timeout.connect(drain_ros_callbacks)
    spin_timer.start()

    signal.signal(signal.SIGINT, lambda *_: app.quit())
    signal_timer = QTimer()
    signal_timer.setInterval(250)
    signal_timer.timeout.connect(lambda: None)
    signal_timer.start()

    window.append_log('info', f'Qt binding: {QT_BINDING}')
    window.show()

    exit_code = 1
    try:
        exit_code = app_exec(app)
    finally:
        spin_timer.stop()
        signal_timer.stop()
        node.close()
        deadline = time.monotonic() + 1.0
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return exit_code


def main(args=None) -> None:
    del args
    raise SystemExit(_run_ros(sys.argv))


if __name__ == '__main__':
    main()
