"""Planner Tasks loaded by the planner Scenario UI.

Only entries returned by :func:`build_tasks` appear in the UI. Other motion
helpers remain composable but are not registered as standalone operator Tasks.
"""
from __future__ import annotations

from typing import Sequence

from .task_commands import RunnableTaskDefinition, Task


# Task-authoring units match rby1_control: seconds, metres, degrees and the
# native velocity/acceleration units documented by Task commands.
#(minimum_time_sec, linear_velocity, linear_acceleration, angular_velocity)
_PICKUP_TCP_MOTION = (5.0, 0.20, 0.20, 0.30)
_PICKUP_DISTANCE_M = 0.10

# Mock tag_0 faces upward. Stop ee_right 10 cm above it and retain the measured
# downward-facing pickup orientation. Replace these values with the calibrated
# object<-EE grasp transform before using this Task on a real object.
_OBJECT_TO_RIGHT_EE_POSITION = (0.0, 0.0, _PICKUP_DISTANCE_M)
_OBJECT_TO_RIGHT_EE_ORIENTATION_XYZW = (
    0.0001741319553783714,
    -0.00011820929270613936,
    0.7064893671706711,
    0.7077236252799605,
)


def object_gripping_initial_pose() -> Task:
    """Move to the measured initial posture used by the handover demo."""

    task = Task(
        'object_gripping_initial_pose',
        'Measured initial posture with both grippers facing downward.',
    )
    task.whole_body_joint_absolute(
        torso=(
            0.059126,
            45.376237,
            -90.963053,
            45.154560,
            0.021390,
            -0.000280,
        ),
        right_arm=(
            -34.913810,
            -63.837794,
            46.172754,
            -106.640370,
            16.002640,
            45.214678,
            83.061208,
        ),
        left_arm=(
            -33.643824,
            63.218668,
            -44.626985,
            -106.563932,
            -17.501234,
            44.556740,
            -84.095907,
        ),
        joint_motion=(2.0, 2.0, 2.0),
    )
    return task


def torso_rotation() -> Task:
    """Rotate the torso to the measured handover placement posture."""

    task = Task('torso_rotation')
    task.joint_absolute(
        'torso',
        (
            0.065071,
            45.375357,
            -90.960025,
            45.153579,
            0.027261,
            -90.0,
        ),
        (4.0, 1.0, 1.0),
    )
    return task


def move_right_ee_to_detected_object(
    *,
    object_id: str,
    tcp_motion: Sequence[object],
    object_to_ee_position: Sequence[object],
    object_to_ee_orientation_xyzw: Sequence[object],
    detection_timeout_sec: float = 3.0,
    max_age_sec: float | None = None,
    minimum_confidence: float | None = None,
) -> RunnableTaskDefinition:
    """Build a right-EE move whose target is captured during execution."""

    task = Task(
        'move_right_ee_to_detected_object',
        f'Move right EE to a newly detected {object_id} Cartesian pose.',
    )
    task.delay(2000)
    task.camera_linear_absolute(
        'right_arm',
        object_id,
        tcp_motion,
        detection_timeout_sec=detection_timeout_sec,
        max_age_sec=max_age_sec,
        minimum_confidence=minimum_confidence,
        object_to_end_effector_position=object_to_ee_position,
        object_to_end_effector_orientation_xyzw=(
            object_to_ee_orientation_xyzw
        ),
    )
    return task.build()


def _configured_detected_object_move() -> RunnableTaskDefinition:
    return move_right_ee_to_detected_object(
        object_id='tag_0',
        tcp_motion=_PICKUP_TCP_MOTION,
        object_to_ee_position=_OBJECT_TO_RIGHT_EE_POSITION,
        object_to_ee_orientation_xyzw=(
            _OBJECT_TO_RIGHT_EE_ORIENTATION_XYZW
        ),
        detection_timeout_sec=3.0,
    )


def object_handover_demo2() -> RunnableTaskDefinition:
    """Detect, pick, rotate, and place an object with the right hand."""

    task = Task(
        'object_handover_demo2',
        'Capture tag_0 after the initial pose, then perform the handover demo.',
    )
    task.extend(object_gripping_initial_pose())

    # Camera lookup is deferred. The runner captures a new tag observation
    # only after the initial whole-body motion has completed and become idle.
    task.extend(_configured_detected_object_move())

    task.delay(750)  # Placeholder: close the right gripper.
    task.linear_relative(
        'right_arm',
        (0.0, 0.0, _PICKUP_DISTANCE_M, 0.0, 0.0, 0.0),
        _PICKUP_TCP_MOTION,
    )
    task.extend(torso_rotation())
    task.linear_relative(
        'right_arm',
        (0.0, 0.0, -_PICKUP_DISTANCE_M, 0.0, 0.0, 0.0),
        _PICKUP_TCP_MOTION,
    )
    task.delay(750)  # Placeholder: open the right gripper.
    task.linear_relative(
        'right_arm',
        (0.0, 0.0, _PICKUP_DISTANCE_M, 0.0, 0.0, 0.0),
        _PICKUP_TCP_MOTION,
    )
    return task.build()

def ready_pose() -> Task:
    """Move torso and both arms together to the Cartesian-ready posture."""

    task = Task(
        "ready_pose",
        "Moves torso and both arms together to the SDK Cartesian-ready pose.",
    )
    joint_motion = [5.0, 0.20, 0.20]
    task.whole_body_joint_absolute(
        torso=[0.0, 45.0, -90.0, 45.0, 0.0, 0.0],
        right_arm=[0.0, -5.0, 0.0, -120.0, 0.0, 40.0, 0.0],
        left_arm=[0.0, 5.0, 0.0, -120.0, 0.0, 40.0, 0.0],
        joint_motion=joint_motion,
    )
    return task
    
def build_tasks() -> dict[str, RunnableTaskDefinition]:
    """Return only the Tasks shown in the planner Scenario Task List."""

    return {
        'move_right_ee_to_detected_object': _configured_detected_object_move(),
        'object_handover_demo2': object_handover_demo2(),
        'object_gripping_initial_pose': object_gripping_initial_pose().build(),
        'ready_pose': ready_pose().build(),
    }


__all__ = [
    'Task',
    'build_tasks',
    'move_right_ee_to_detected_object',
    'object_gripping_initial_pose',
    'object_handover_demo2',
    'torso_rotation',
    'ready_pose',
]
