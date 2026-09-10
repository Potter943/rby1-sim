"""Versioned JSON protocol used between the UI and ROS backend nodes.

The transport intentionally uses ``std_msgs/String`` so control clients do not
need a second interface-generation package. Keeping all encoding here gives the
planner one documented, versioned command schema instead of depending on Python
method calls.
"""
from __future__ import annotations

import json
from typing import Any, Mapping, Optional

from .backend_contract import (
    BackendSnapshot,
    TaskBackendState,
    TaskCommandState,
    TaskCommandStatus,
    VelocityCommand,
)
from .task_commands import CommandKind, TaskCommand


PROTOCOL_VERSION = 1
COMMAND_KIND = "command"
STATE_KIND = "state"
EVENT_KIND = "event"
RESPONSE_KIND = "response"


def _reject_json_constant(value: str):
    raise ValueError(f"non-finite JSON number is not allowed: {value}")


def encode_message(kind: str, payload: Mapping[str, Any]) -> str:
    """Encode one protocol envelope and reject non-standard JSON values."""

    envelope = dict(payload)
    envelope["schema_version"] = PROTOCOL_VERSION
    envelope["kind"] = str(kind)
    return json.dumps(
        envelope,
        ensure_ascii=False,
        separators=(",", ":"),
        allow_nan=False,
    )


def decode_message(raw: str, *, expected_kind: Optional[str] = None) -> dict:
    """Decode and minimally validate one protocol envelope."""

    try:
        envelope = json.loads(raw, parse_constant=_reject_json_constant)
    except (TypeError, json.JSONDecodeError) as exc:
        raise ValueError("transport payload is not valid JSON") from exc

    if not isinstance(envelope, dict):
        raise ValueError("transport payload must be a JSON object")
    if envelope.get("schema_version") != PROTOCOL_VERSION:
        raise ValueError("unsupported control transport schema version")
    if not isinstance(envelope.get("kind"), str):
        raise ValueError("transport payload kind is missing")
    if expected_kind is not None and envelope["kind"] != expected_kind:
        raise ValueError(
            f"expected {expected_kind!r} payload, got {envelope['kind']!r}"
        )
    return envelope


def task_command_to_dict(command: TaskCommand) -> dict:
    """Convert an immutable Task command to its wire representation."""

    if not isinstance(command, TaskCommand):
        raise TypeError("command must be a TaskCommand")

    payload = {
        "kind": command.kind.value,
        "group": command.group,
        "values": list(command.values),
        "joint_targets": {
            group: list(values)
            for group, values in command.joint_targets
        },
        "minimum_time": command.minimum_time,
        "velocity_limit": command.velocity_limit,
        "acceleration_limit": command.acceleration_limit,
        "linear_velocity": command.linear_velocity,
        "angular_velocity": command.angular_velocity,
        "acceleration_scaling": command.acceleration_scaling,
        "seconds": command.seconds,
    }
    return {
        key: value
        for key, value in payload.items()
        if value is not None and value != {}
    }


def task_command_from_dict(payload: Mapping[str, Any]) -> TaskCommand:
    """Build and validate a Task command received over the command topic."""

    if not isinstance(payload, Mapping):
        raise ValueError("Task command payload must be an object")

    raw_targets = payload.get("joint_targets", {})
    if not isinstance(raw_targets, Mapping):
        raise ValueError("joint_targets must be an object")

    return TaskCommand(
        kind=CommandKind(str(payload.get("kind", ""))),
        group=(
            str(payload["group"])
            if payload.get("group") is not None
            else None
        ),
        values=tuple(payload.get("values", ())),
        joint_targets=tuple(
            (str(group), tuple(values))
            for group, values in raw_targets.items()
        ),
        minimum_time=payload.get("minimum_time"),
        velocity_limit=payload.get("velocity_limit"),
        acceleration_limit=payload.get("acceleration_limit"),
        linear_velocity=payload.get("linear_velocity"),
        angular_velocity=payload.get("angular_velocity"),
        acceleration_scaling=payload.get("acceleration_scaling"),
        seconds=payload.get("seconds"),
    )


def backend_snapshot_to_dict(snapshot: BackendSnapshot) -> dict:
    return {
        "namespace": snapshot.namespace,
        "cmd_vel_topic": snapshot.cmd_vel_topic,
        "cmd_vel_subscribers": snapshot.cmd_vel_subscribers,
        "control_state": snapshot.control_state,
        "power_enabled": snapshot.power_enabled,
        "servo_enabled": snapshot.servo_enabled,
        "stream_enabled": snapshot.stream_enabled,
        "emo_active": snapshot.emo_active,
        "collision_active": snapshot.collision_active,
        "services_enabled": snapshot.services_enabled,
        "rby1_msgs_available": snapshot.rby1_msgs_available,
        "service_ready": dict(snapshot.service_ready),
        "command": {
            "vx": snapshot.command.vx,
            "vy": snapshot.command.vy,
            "wz": snapshot.command.wz,
        },
        "command_stale": snapshot.command_stale,
    }


def backend_snapshot_from_dict(payload: Mapping[str, Any]) -> BackendSnapshot:
    command = payload.get("command", {})
    if not isinstance(command, Mapping):
        command = {}
    readiness = payload.get("service_ready", {})
    if not isinstance(readiness, Mapping):
        readiness = {}

    return BackendSnapshot(
        namespace=str(payload.get("namespace", "")),
        cmd_vel_topic=str(payload.get("cmd_vel_topic", "")),
        cmd_vel_subscribers=int(payload.get("cmd_vel_subscribers", 0)),
        control_state=_optional_int(payload.get("control_state")),
        power_enabled=_optional_bool(payload.get("power_enabled")),
        servo_enabled=_optional_bool(payload.get("servo_enabled")),
        stream_enabled=_optional_bool(payload.get("stream_enabled")),
        emo_active=_optional_bool(payload.get("emo_active")),
        collision_active=_optional_bool(payload.get("collision_active")),
        services_enabled=bool(payload.get("services_enabled", False)),
        rby1_msgs_available=bool(payload.get("rby1_msgs_available", False)),
        service_ready={str(k): bool(v) for k, v in readiness.items()},
        command=VelocityCommand(
            float(command.get("vx", 0.0)),
            float(command.get("vy", 0.0)),
            float(command.get("wz", 0.0)),
        ),
        command_stale=bool(payload.get("command_stale", True)),
    )


def task_backend_state_to_dict(state: TaskBackendState) -> dict:
    return {
        "captured_at": state.captured_at,
        "robot_state_updated_at": state.robot_state_updated_at,
        "control_state": state.control_state,
        "stream_enabled": state.stream_enabled,
        "emo_active": state.emo_active,
        "collision_active": state.collision_active,
        "motion_active": state.motion_active,
        "joint_groups": _values_mapping_to_dict(state.joint_groups),
        "joint_updated_at": dict(state.joint_updated_at),
        "joint_order_verified": dict(state.joint_order_verified),
        "cartesian": _values_mapping_to_dict(state.cartesian),
        "cartesian_updated_at": dict(state.cartesian_updated_at),
        "driver_safety_verified": state.driver_safety_verified,
        "driver_safety_updated_at": state.driver_safety_updated_at,
    }


def task_backend_state_from_dict(payload: Mapping[str, Any]) -> TaskBackendState:
    return TaskBackendState(
        captured_at=float(payload.get("captured_at", 0.0)),
        robot_state_updated_at=_optional_float(
            payload.get("robot_state_updated_at")
        ),
        control_state=_optional_int(payload.get("control_state")),
        stream_enabled=_optional_bool(payload.get("stream_enabled")),
        emo_active=_optional_bool(payload.get("emo_active")),
        collision_active=_optional_bool(payload.get("collision_active")),
        motion_active=bool(payload.get("motion_active", False)),
        joint_groups=_values_mapping_from_dict(payload.get("joint_groups", {})),
        joint_updated_at=_optional_float_mapping(
            payload.get("joint_updated_at", {})
        ),
        joint_order_verified={
            str(key): bool(value)
            for key, value in _mapping(payload.get("joint_order_verified", {})).items()
        },
        cartesian=_values_mapping_from_dict(payload.get("cartesian", {})),
        cartesian_updated_at=_optional_float_mapping(
            payload.get("cartesian_updated_at", {})
        ),
        driver_safety_verified=bool(
            payload.get("driver_safety_verified", False)
        ),
        driver_safety_updated_at=_optional_float(
            payload.get("driver_safety_updated_at")
        ),
    )


def task_command_state_to_dict(state: TaskCommandState) -> dict:
    return {
        "status": state.status.value,
        "message": state.message,
    }


def task_command_state_from_dict(payload: Mapping[str, Any]) -> TaskCommandState:
    return TaskCommandState(
        status=TaskCommandStatus(str(payload.get("status", "pending"))),
        message=str(payload.get("message", "")),
    )


def _mapping(value: Any) -> Mapping[str, Any]:
    return value if isinstance(value, Mapping) else {}


def _values_mapping_to_dict(values: Mapping[str, Any]) -> dict:
    return {
        str(key): (list(value) if value is not None else None)
        for key, value in values.items()
    }


def _values_mapping_from_dict(value: Any) -> dict:
    return {
        str(key): (tuple(item) if item is not None else None)
        for key, item in _mapping(value).items()
    }


def _optional_float_mapping(value: Any) -> dict:
    return {
        str(key): _optional_float(item)
        for key, item in _mapping(value).items()
    }


def _optional_float(value: Any) -> Optional[float]:
    return None if value is None else float(value)


def _optional_int(value: Any) -> Optional[int]:
    return None if value is None else int(value)


def _optional_bool(value: Any) -> Optional[bool]:
    return None if value is None else bool(value)
