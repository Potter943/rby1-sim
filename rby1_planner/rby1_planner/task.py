"""Runtime Task factories that turn validated observations into commands.

This module deliberately contains no assumed grasp geometry or gripper action.
Callers must provide the arm, object-frame end-effector offset, and every motion
limit explicitly after applying their own workspace and collision policy.
"""
from __future__ import annotations

from typing import Iterable, Sequence

from .observation import (
    ObjectObservation,
    compose_pose_right,
    observation_to_cartesian_target,
)
from .task_commands import CARTESIAN_ARMS, Task, TaskDefinition


def build_observation_cartesian_task(
    *,
    name: str,
    observation: ObjectObservation,
    arm: str,
    object_to_end_effector_position: Iterable[object],
    object_to_end_effector_orientation_xyzw: Iterable[object],
    tcp_motion: Sequence[object],
    target_frame: str = 'base',
    description: str = '',
) -> TaskDefinition:
    """Build one absolute Cartesian command from an object observation.

    The observation must already be expressed in ``target_frame``.  The goal is
    computed as ``target_frame<-object * object<-end_effector``.  No offset or
    motion setting has a default because those values are application- and
    hardware-specific safety inputs.
    """

    arm_name = str(arm).strip()
    if arm_name not in CARTESIAN_ARMS:
        raise ValueError(
            f'arm must be one of {CARTESIAN_ARMS}; got {arm_name!r}'
        )
    if not isinstance(observation, ObjectObservation):
        raise TypeError('observation must be an ObjectObservation')

    goal_position, goal_orientation = compose_pose_right(
        observation.position,
        observation.orientation_xyzw,
        object_to_end_effector_position,
        object_to_end_effector_orientation_xyzw,
    )
    goal_observation = ObjectObservation(
        object_id=observation.object_id,
        frame_id=observation.frame_id,
        stamp_ns=observation.stamp_ns,
        received_at_ns=observation.received_at_ns,
        position=goal_position,
        orientation_xyzw=goal_orientation,
        confidence=observation.confidence,
    )
    target = observation_to_cartesian_target(
        goal_observation,
        target_frame=target_frame,
    )

    task = Task(name, description)
    task.linear_absolute(arm_name, target, tcp_motion)
    return task.build()


__all__ = ['build_observation_cartesian_task']
