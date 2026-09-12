"""Headless camera-aware frontend for the RB-Y1 control backend."""
from __future__ import annotations

from typing import Optional

from rby1_control.frontend_node import ControlFrontendNode

from .camera import CameraClient
from .task_commands import RunnableTaskDefinition
from .task_runner import PlannerTaskRunner


class PlannerNode(ControlFrontendNode):
    """Share the production control protocol and add planner-side perception.

    Construction only creates ROS publishers/subscribers and timers. It does
    not prepare the robot, enable its stream, or send any motion command.
    """

    def __init__(
        self,
        *,
        namespace: Optional[str] = 'rby1',
        tf_buffer=None,
    ) -> None:
        super().__init__(node_name='rby1_planner', namespace=namespace)

        self.declare_parameter('object_pose_topic', 'perception/object_pose')
        self.declare_parameter('object_pose_target_frame', 'base')
        self.declare_parameter('object_pose_max_age_sec', 0.5)
        self.declare_parameter('object_pose_min_confidence', 0.5)
        self.declare_parameter('object_pose_future_tolerance_sec', 0.05)
        self.declare_parameter('object_pose_tf_timeout_sec', 0.0)

        self.camera = CameraClient(
            self,
            object_pose_topic=str(
                self.get_parameter('object_pose_topic').value
            ),
            target_frame=str(
                self.get_parameter('object_pose_target_frame').value
            ),
            max_age_sec=float(
                self.get_parameter('object_pose_max_age_sec').value
            ),
            minimum_confidence=float(
                self.get_parameter('object_pose_min_confidence').value
            ),
            future_tolerance_sec=float(
                self.get_parameter(
                    'object_pose_future_tolerance_sec'
                ).value
            ),
            tf_timeout_sec=float(
                self.get_parameter('object_pose_tf_timeout_sec').value
            ),
            tf_buffer=tf_buffer,
        )
        self.task_runner = PlannerTaskRunner(
            self,
            self,
            camera=self.camera,
            on_status=self._task_status,
        )
        self.get_logger().info(
            'Planner frontend is ready and idle; no automatic task is configured.'
        )

    def run_task(self, task: RunnableTaskDefinition) -> None:
        """Start one explicitly supplied Task through the control backend."""

        self.task_runner.start(task)

    def stop_task(self, reason: str = 'Planner Task stopped') -> None:
        self.task_runner.stop(reason)

    def close(self) -> None:
        """Cancel planner work and request a motion-safe backend shutdown."""

        try:
            self.task_runner.close()
        finally:
            # This stops base/manipulator motion and turns Stream off, but does
            # not power off the robot or disable its servos.
            self.shutdown_safely(turn_stream_off=True)

    def _task_status(self, message: str) -> None:
        self.get_logger().info(message)


__all__ = ['PlannerNode']
