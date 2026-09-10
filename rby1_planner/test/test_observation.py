from dataclasses import FrozenInstanceError
import math

import pytest

from rby1_planner.observation import (
    ObjectObservation,
    compose_pose_right,
    normalize_quaternion,
    observation_to_cartesian_target,
    quaternion_to_rpy_deg,
    transform_observation,
)


def _observation(**overrides):
    values = {
        'object_id': 'tag_0',
        'frame_id': 'camera',
        'stamp_ns': 1_000_000_000,
        'received_at_ns': 1_100_000_000,
        'position': (1.0, 0.0, 0.0),
        'orientation_xyzw': (0.0, 0.0, 0.0, 1.0),
        'confidence': 0.9,
    }
    values.update(overrides)
    return ObjectObservation(**values)


def test_observation_is_deeply_immutable_and_normalized():
    observation = _observation(
        position=[1, 2, 3],
        orientation_xyzw=[0, 0, 0, 1.0005],
    )

    assert observation.position == (1.0, 2.0, 3.0)
    assert observation.orientation_xyzw == (0.0, 0.0, 0.0, 1.0)
    with pytest.raises(FrozenInstanceError):
        observation.object_id = 'changed'
    with pytest.raises(TypeError):
        observation.position[0] = 9.0


@pytest.mark.parametrize(
    'field,value,match',
    [
        ('object_id', '', 'object_id'),
        ('frame_id', ' ', 'frame_id'),
        ('position', (math.nan, 0, 0), 'finite'),
        ('orientation_xyzw', (0, 0, 0, 0), 'zero quaternion'),
        ('orientation_xyzw', (0, 0, 0, 2), 'normalized'),
        ('confidence', 1.1, 'confidence'),
        ('stamp_ns', -1, 'stamp_ns'),
    ],
)
def test_observation_rejects_invalid_fields(field, value, match):
    with pytest.raises(ValueError, match=match):
        _observation(**{field: value})


def test_transform_observation_composes_translation_and_rotation():
    # target <- camera rotates +90 degrees about Z and translates by +Y.
    half = math.sqrt(0.5)
    transformed = transform_observation(
        _observation(),
        target_frame='base',
        translation=(0.0, 1.0, 0.0),
        rotation_xyzw=(0.0, 0.0, half, half),
    )

    assert transformed.frame_id == 'base'
    assert transformed.position == pytest.approx((0.0, 2.0, 0.0))
    assert transformed.orientation_xyzw == pytest.approx(
        (0.0, 0.0, half, half)
    )
    assert transformed.object_id == 'tag_0'
    assert transformed.stamp_ns == 1_000_000_000


def test_quaternion_validation_rejects_non_finite_values():
    with pytest.raises(ValueError, match='finite'):
        normalize_quaternion((0.0, 0.0, math.inf, 1.0))


def test_identity_observation_converts_to_backend_cartesian_target():
    observation = _observation(
        frame_id='base',
        position=(0.3, -0.2, 0.8),
    )

    assert quaternion_to_rpy_deg(observation.orientation_xyzw) == pytest.approx(
        (0.0, 0.0, 0.0)
    )
    assert observation_to_cartesian_target(observation) == pytest.approx(
        (0.3, -0.2, 0.8, 0.0, 0.0, 0.0)
    )


def test_quaternion_to_rpy_matches_backend_convention_for_yaw_90_degrees():
    half = math.sqrt(0.5)

    assert quaternion_to_rpy_deg((0.0, 0.0, half, half)) == pytest.approx(
        (0.0, 0.0, 90.0)
    )


def test_quaternion_to_rpy_clamps_pitch_at_gimbal_boundary():
    half = math.sqrt(0.5)
    roll, pitch, yaw = quaternion_to_rpy_deg((0.0, half, 0.0, half))

    assert all(math.isfinite(value) for value in (roll, pitch, yaw))
    assert pitch == pytest.approx(90.0)


def test_compose_pose_right_rotates_object_frame_offset():
    half = math.sqrt(0.5)

    position, orientation = compose_pose_right(
        (1.0, 2.0, 3.0),
        (0.0, 0.0, half, half),
        (1.0, 0.0, 0.0),
        (0.0, 0.0, 0.0, 1.0),
    )

    assert position == pytest.approx((1.0, 3.0, 3.0))
    assert orientation == pytest.approx((0.0, 0.0, half, half))


def test_compose_pose_right_applies_orientation_offset_on_the_right():
    half = math.sqrt(0.5)

    _position, orientation = compose_pose_right(
        (0.0, 0.0, 0.0),
        (0.0, 0.0, half, half),  # base <- object: +90 yaw
        (0.0, 0.0, 0.0),
        (half, 0.0, 0.0, half),  # object <- EE: +90 roll
    )

    assert orientation == pytest.approx((0.5, 0.5, 0.5, 0.5))


def test_observation_to_cartesian_target_requires_matching_frame():
    with pytest.raises(ValueError, match='does not match'):
        observation_to_cartesian_target(
            _observation(frame_id='camera'),
            target_frame='base',
        )
