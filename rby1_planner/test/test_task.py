import pytest

from rby1_control.task_commands import CommandKind, TaskDefinition
from rby1_planner.task import (
    build_tasks,
    move_right_ee_to_detected_object,
    object_gripping_initial_pose,
    object_handover_demo2,
    torso_rotation,
)
from rby1_planner.task_commands import (
    CameraLinearAbsoluteStep,
    DynamicTaskDefinition,
    Task,
)
from rby1_planner.task_registry import load_tasks_from_source


def test_initial_pose_preserves_the_measured_whole_body_target():
    definition = object_gripping_initial_pose().build()

    assert isinstance(definition, TaskDefinition)
    assert len(definition.commands) == 1
    command = definition.commands[0]
    assert command.kind is CommandKind.JOINT_ABSOLUTE_MULTI
    targets = dict(command.joint_targets)
    assert targets['torso'] == pytest.approx((
        0.059126,
        45.376237,
        -90.963053,
        45.154560,
        0.021390,
        -0.000280,
    ))
    assert targets['right_arm'] == pytest.approx((
        -34.913810,
        -63.837794,
        46.172754,
        -106.640370,
        16.002640,
        45.214678,
        83.061208,
    ))
    assert targets['left_arm'] == pytest.approx((
        -33.643824,
        63.218668,
        -44.626985,
        -106.563932,
        -17.501234,
        44.556740,
        -84.095907,
    ))


def test_torso_rotation_preserves_the_measured_target():
    definition = torso_rotation().build()

    assert isinstance(definition, TaskDefinition)
    command = definition.commands[0]
    assert command.kind is CommandKind.JOINT_ABSOLUTE
    assert command.group == 'torso'
    assert command.values == pytest.approx((
        0.065071,
        45.375357,
        -90.960025,
        45.153579,
        0.027261,
        -90.0,
    ))


def test_camera_motion_factory_builds_a_deferred_planner_step():
    definition = move_right_ee_to_detected_object(
        object_id='tag_7',
        tcp_motion=(2.0, 0.05, 0.2, 0.2),
        object_to_ee_position=(0.0, 0.0, 0.1),
        object_to_ee_orientation_xyzw=(0.0, 0.0, 0.0, 1.0),
        detection_timeout_sec=4.0,
        max_age_sec=0.25,
        minimum_confidence=0.8,
    )

    assert isinstance(definition, DynamicTaskDefinition)
    assert len(definition.commands) == 1
    step = definition.commands[0]
    assert isinstance(step, CameraLinearAbsoluteStep)
    assert step.group == 'right_arm'
    assert step.object_id == 'tag_7'
    assert step.detection_timeout_sec == pytest.approx(4.0)
    assert step.max_age_sec == pytest.approx(0.25)
    assert step.minimum_confidence == pytest.approx(0.8)
    assert step.object_to_end_effector_position == (0.0, 0.0, 0.1)
    assert step.object_to_end_effector_orientation_xyzw == (
        0.0,
        0.0,
        0.0,
        1.0,
    )


def test_handover_demo_places_the_dynamic_camera_step_between_motions():
    definition = object_handover_demo2()

    assert isinstance(definition, DynamicTaskDefinition)
    assert len(definition.commands) == 8
    assert definition.commands[0].kind is CommandKind.JOINT_ABSOLUTE_MULTI
    assert isinstance(definition.commands[1], CameraLinearAbsoluteStep)
    assert definition.commands[2].kind is CommandKind.DELAY
    assert definition.commands[3].kind is CommandKind.LINEAR_RELATIVE
    assert definition.commands[3].values == (
        0.0,
        0.0,
        0.1,
        0.0,
        0.0,
        0.0,
    )
    assert definition.commands[4].kind is CommandKind.JOINT_ABSOLUTE
    assert definition.commands[4].group == 'torso'
    assert definition.commands[5].kind is CommandKind.LINEAR_RELATIVE
    assert definition.commands[5].values == (
        0.0,
        0.0,
        -0.1,
        0.0,
        0.0,
        0.0,
    )
    assert definition.commands[6].kind is CommandKind.DELAY
    assert definition.commands[7].kind is CommandKind.LINEAR_RELATIVE


def test_configured_camera_move_uses_safe_mock_approach_pose():
    step = build_tasks()['move_right_ee_to_detected_object'].commands[0]

    assert isinstance(step, CameraLinearAbsoluteStep)
    assert step.object_to_end_effector_position == (0.0, 0.0, 0.1)
    assert step.object_to_end_effector_orientation_xyzw == pytest.approx((
        0.0001741319553783714,
        -0.00011820929270613936,
        0.7064893671706711,
        0.7077236252799605,
    ))
    assert step.minimum_time == pytest.approx(5.0)
    assert step.linear_velocity == pytest.approx(0.05)
    assert step.angular_velocity == pytest.approx(0.20)
    assert step.acceleration_scaling == pytest.approx(0.30)


def test_build_tasks_exposes_registered_operator_tasks():
    tasks = build_tasks()

    assert set(tasks) == {
        'move_right_ee_to_detected_object',
        'object_handover_demo2',
        'object_gripping_initial_pose',
    }
    assert all(name == task.name for name, task in tasks.items())


def test_planner_registry_reloads_dynamic_tasks_without_serializing_them():
    tasks = load_tasks_from_source()

    assert set(tasks) == {
        'move_right_ee_to_detected_object',
        'object_handover_demo2',
        'object_gripping_initial_pose',
    }
    assert isinstance(
        tasks['move_right_ee_to_detected_object'],
        DynamicTaskDefinition,
    )
    assert isinstance(tasks['object_handover_demo2'], DynamicTaskDefinition)
    assert isinstance(tasks['object_gripping_initial_pose'], TaskDefinition)


@pytest.mark.parametrize(
    'kwargs,match',
    [
        ({'object_id': ''}, 'object_id'),
        ({'detection_timeout_sec': 0.0}, 'positive finite'),
        ({'max_age_sec': -1.0}, 'positive finite'),
        ({'minimum_confidence': 1.1}, 'range'),
        (
            {'object_to_end_effector_position': (0.0, 0.0)},
            '3 finite numbers',
        ),
        (
            {
                'object_to_end_effector_orientation_xyzw': (
                    0.0,
                    0.0,
                    0.0,
                    0.0,
                )
            },
            'zero quaternion',
        ),
    ],
)
def test_camera_step_rejects_invalid_perception_settings(kwargs, match):
    values = {
        'arm': 'right_arm',
        'object_id': 'tag_0',
        'tcp_motion': (1.0, 0.1, 0.2, 0.5),
    }
    values.update(kwargs)

    with pytest.raises(ValueError, match=match):
        Task('invalid').camera_linear_absolute(**values)
