from pathlib import Path

import pytest

from rby1_control.task_commands import (
    CommandKind,
    Task,
    delete_catalog_task,
    list_sum,
    load_catalog,
    save_catalog,
)


def test_list_sum_is_strict_and_elementwise():
    assert list_sum([1, 2, 3], [0.5, -2, 4]) == [1.5, 0.0, 7.0]
    with pytest.raises(ValueError):
        list_sum([1], [1, 2])
    with pytest.raises(ValueError):
        list_sum([True], [1])


def test_task_keeps_explicit_driver_motion_values():
    task = Task("tcp")
    task.linear_absolute(
        "right_arm",
        [0.3, -0.2, 0.8, 0, 0, 0],
        [2.0, 0.05, 0.2, 0.2],
    )
    command = task.build().commands[0]
    assert command.kind is CommandKind.LINEAR_ABSOLUTE
    assert command.minimum_time == pytest.approx(2.0)
    assert command.linear_velocity == pytest.approx(0.05)
    assert command.angular_velocity == pytest.approx(0.2)
    assert command.acceleration_scaling == pytest.approx(0.2)


def test_task_extend_and_catalog_round_trip(tmp_path: Path):
    first = Task("first").delay(100)
    combined = Task("combined").extend(first).extend(first).build()
    assert len(combined.commands) == 2

    catalog = tmp_path / "catalog.json"
    save_catalog({combined.name: combined}, catalog)
    loaded = load_catalog(catalog)
    assert loaded == {"combined": combined}

    assert delete_catalog_task("combined", catalog) == {}
    assert load_catalog(catalog) == {}


def test_whole_body_joint_command_round_trip(tmp_path: Path):
    task = Task("ready")
    task.whole_body_joint_absolute(
        torso=[0, 10, -20, 10, 0, 0],
        right_arm=[0, -5, 0, -90, 0, 40, 0],
        left_arm=[0, 5, 0, -90, 0, 40, 0],
        joint_motion=[8.0, 0.2, 0.2],
    )
    definition = task.build()
    command = definition.commands[0]
    assert command.kind is CommandKind.JOINT_ABSOLUTE_MULTI
    assert tuple(group for group, _values in command.joint_targets) == (
        "torso",
        "right_arm",
        "left_arm",
    )

    catalog = tmp_path / "catalog.json"
    save_catalog({definition.name: definition}, catalog)
    assert load_catalog(catalog) == {definition.name: definition}


def test_multi_joint_command_requires_supported_body_groups():
    with pytest.raises(ValueError, match="at least two"):
        Task("bad").joint_absolute_multi(
            {"right_arm": [0] * 7},
            [2.0, 0.2, 0.2],
        )

    with pytest.raises(ValueError, match="unsupported"):
        Task("bad").joint_absolute_multi(
            {"right_arm": [0] * 7, "head": [0] * 2},
            [2.0, 0.2, 0.2],
        )


def test_invalid_scaling_is_rejected():
    with pytest.raises(ValueError, match="cannot exceed"):
        Task("bad").linear_relative(
            "left_arm",
            [0, 0, 0, 0, 0, 0],
            [1, 0.05, 0.2, 1.1],
        )


def test_tcp_motion_requires_exactly_four_values():
    with pytest.raises(ValueError, match="requires 4 values"):
        Task("bad").linear_absolute(
            "right_arm",
            [0.3, -0.2, 0.8, 0, 0, 0],
            [2.0, 0.05, 0.2],
        )


def test_joint_motion_requires_exactly_three_values():
    with pytest.raises(ValueError, match="requires 3 values"):
        Task("bad").joint_absolute(
            "right_arm",
            [0] * 7,
            [2.0, 0.2],
        )
