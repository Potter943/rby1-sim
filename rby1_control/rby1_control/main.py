"""Application entry point for the RB-Y1 control debug UI."""
from __future__ import annotations

import signal
import sys
import time

from .qt_compat import QApplication, QTimer, QT_BINDING, app_exec
from .main_window import MainWindow


def _run_ros(argv):
    import rclpy
    from .frontend_node import ControlUiFrontendNode

    rclpy.init(args=argv[1:])
    app = QApplication(argv)
    app.setApplicationName('RB-Y1 M v1.3 Control Debug UI')

    node = ControlUiFrontendNode()
    window = MainWindow(node)
    app.installEventFilter(window)

    spin_timer = QTimer()
    spin_timer.setInterval(10)
    spin_timer.timeout.connect(lambda: rclpy.spin_once(node, timeout_sec=0.0))
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
        node.shutdown_safely(turn_stream_off=True)
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
