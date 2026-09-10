import math

import pytest

from rby1_control.task_commands import (
    CommandKind,
    TaskDefinition as CanonicalTaskDefinition,
)
from rby1_planner.observation import ObjectObservation
from rby1_planner.task import build_observation_cartesian_task
from rby1_planner.task_commands import TaskDefinition as PlannerTaskDefinition


def _observation(**overrides):
    values = {
        'object_id': 'workpiece',
        'frame_id': 'base',
        'stamp_ns': 1_000_000_000,
        'received_at_ns': 1_100_000_000,
        'position': (1.0, 2.0, 3.0),
        'orientation_xyzw': (0.0, 0.0, 0.0, 1.0),
        'confidence': 0.95,
    }
    values.update(overrides)
    return ObjectObservation(**values)


def _build(**overrides):
    values = {
        'name': 'move-to-observation',
        'observation': _observation(),
        'arm': 'right_arm',
        'object_to_end_effector_position': (0.0, 0.0, 0.0),
        'object_to_end_effector_orientation_xyzw': (0.0, 0.0, 0.0, 1.0),
        'tcp_motion': (2.0, 0.05, 0.2, 0.2),
    }
    values.update(overrides)
    return build_observation_cartesian_task(**values)


def test_factory_returns_canonical_task_definition():
    definition = _build()

    assert PlannerTaskDefinition is CanonicalTaskDefinition
    assert isinstance(definition, CanonicalTaskDefinition)
    assert len(definition.commands) == 1
    command = definition.commands[0]
    assert command.kind is CommandKind.LINEAR_ABSOLUTE
    assert command.group == 'right_arm'
    assert command.values == pytest.approx((1.0, 2.0, 3.0, 0.0, 0.0, 0.0))
    assert command.minimum_time == pytest.approx(2.0)
    assert command.linear_velocity == pytest.approx(0.05)
    assert command.angular_velocity == pytest.approx(0.2)
    assert command.acceleration_scaling == pytest.approx(0.2)


def test_factory_applies_offset_in_rotated_object_frame():
    half = math.sqrt(0.5)

    definition = _build(
        observation=_observation(
            orientation_xyzw=(0.0, 0.0, half, half),
        ),
        object_to_end_effector_position=(1.0, 0.0, 0.0),
    )

    assert definition.commands[0].values == pytest.approx(
        (1.0, 3.0, 3.0, 0.0, 0.0, 90.0)
    )


def test_factory_rejects_mismatched_observation_frame():
    with pytest.raises(ValueError, match='does not match'):
        _build(
            observation=_observation(frame_id='camera'),
            target_frame='base',
        )


def test_factory_rejects_invalid_arm():
    with pytest.raises(ValueError, match='arm must be one of'):
        _build(arm='head')


@pytest.mark.parametrize(
    'tcp_motion,match',
    [
        ((2.0, 0.05, 0.2), 'requires 4 values'),
        ((0.0, 0.05, 0.2, 0.2), 'positive finite'),
        ((2.0, 0.05, 0.2, 1.1), 'cannot exceed'),
    ],
)
def test_factory_rejects_invalid_motion_values(tcp_motion, match):
    with pytest.raises(ValueError, match=match):
        _build(tcp_motion=tcp_motion)
