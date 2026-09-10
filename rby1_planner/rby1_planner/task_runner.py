"""Qt-free sequential Task execution driven by an ROS node timer."""
from __future__ import annotations

from dataclasses import replace
import math
import time
from typing import Callable, Optional

from rby1_control.backend_contract import TaskBackendState, TaskCommandStatus

from .task_commands import CommandKind, TaskCommand, TaskDefinition, list_sum


class PlannerTaskRunner:
    """Run validated Task commands without blocking the ROS executor."""

    STATE_MAX_AGE_SEC = 1.0
    MOTION_TIMEOUT_MIN_SEC = 10.0
    CARTESIAN_TIMEOUT_MIN_SEC = 30.0
    MOTION_TIMEOUT_SCALE = 3.0
    MOTION_TIMEOUT_MARGIN_SEC = 5.0

    def __init__(
        self,
        node,
        backend,
        *,
        on_status: Optional[Callable[[str], None]] = None,
        on_active_changed: Optional[Callable[[bool], None]] = None,
        clock: Callable[[], float] = time.monotonic,
        period_sec: float = 0.05,
    ) -> None:
        if not math.isfinite(float(period_sec)) or float(period_sec) <= 0.0:
            raise ValueError('period_sec must be a positive finite number')
        self.backend = backend
        self.on_status = on_status or (lambda _message: None)
        self.on_active_changed = on_active_changed or (lambda _active: None)
        self.clock = clock
        self._timer = node.create_timer(float(period_sec), self.tick)
        self._timer.cancel()
        self._task: Optional[TaskDefinition] = None
        self._index = 0
        self._command_id: Optional[str] = None
        self._active_command: Optional[TaskCommand] = None
        self._command_deadline = 0.0
        self._delay_deadline: Optional[float] = None
        self._base_deadline: Optional[float] = None
        self._base_velocity = None
        self._stream_target: Optional[bool] = None
        self._stream_deadline = 0.0

    @property
    def active(self) -> bool:
        return self._task is not None

    @property
    def task_name(self) -> str:
        return self._task.name if self._task is not None else ''

    def start(self, task: TaskDefinition) -> None:
        if self.active:
            raise RuntimeError('another Task is already running')
        if not isinstance(task, TaskDefinition):
            raise TypeError('task must be a TaskDefinition')
        if not task.commands:
            raise ValueError('Task has no commands')
        self._require_safe_state(self.backend.task_state(), require_idle=True)
        self._task = task
        self._index = 0
        self._command_id = None
        self._active_command = None
        self._command_deadline = 0.0
        self._delay_deadline = None
        self._base_deadline = None
        self._base_velocity = None
        self._stream_target = None
        self._stream_deadline = 0.0
        self.on_active_changed(True)
        self.on_status(f'Task started: {task.name}')
        self._timer.reset()
        self.tick()

    def stop(self, reason: str = 'Task stopped') -> None:
        if not self.active:
            return
        try:
            self._cancel_active_motion()
        except Exception as exc:
            self.on_status(f'Task cancel warning: {exc}')
        finally:
            self._finish()
        self.on_status(reason)

    def close(self) -> None:
        if self.active:
            self.stop('Task runner closed')
        self._timer.cancel()

    def tick(self) -> None:
        if self._task is None:
            return
        try:
            state = self.backend.task_state()
            self._require_safe_state(
                state,
                require_idle=self._command_id is None,
            )
            now = self.clock()

            if self._stream_target is not None:
                if state.stream_enabled is self._stream_target:
                    self._stream_target = None
                elif now > self._stream_deadline:
                    raise RuntimeError('Stream state transition timed out')
                else:
                    return

            if self._base_deadline is not None:
                self._tick_base(now)
                return

            if self._delay_deadline is not None:
                if now < self._delay_deadline:
                    return
                self._delay_deadline = None
                self._complete_step()

            if self._command_id is not None:
                command_state = self.backend.poll_task_command(self._command_id)
                if command_state.status is TaskCommandStatus.PENDING:
                    if now > self._command_deadline:
                        raise RuntimeError(
                            'motion command timed out'
                            + self._timeout_detail(state)
                        )
                    return
                if command_state.status is not TaskCommandStatus.SUCCEEDED:
                    detail = command_state.message or command_state.status.value
                    raise RuntimeError(
                        f'motion command did not complete: {detail}'
                    )
                self._command_id = None
                self._active_command = None
                self._complete_step()

            if self._index >= len(self._task.commands):
                name = self._task.name
                self._finish()
                self.on_status(f'Task completed: {name}')
                return

            command = self._task.commands[self._index]
            if command.kind is CommandKind.BASE_VELOCITY:
                if state.stream_enabled is not True:
                    self._request_stream_transition(True, now)
                    return
                self._base_velocity = command.values
                self._base_deadline = now + float(command.seconds or 0.0)
                self.backend.set_velocity(*command.values)
                self.on_status(
                    f'Task step {self._index + 1}/{len(self._task.commands)}: '
                    f'base_velocity {float(command.seconds or 0.0):.3f}s'
                )
                return

            if command.kind is CommandKind.DELAY:
                self._delay_deadline = now + float(command.seconds or 0.0)
                self.on_status(
                    f'Task step {self._index + 1}/{len(self._task.commands)}: '
                    f'delay {float(command.seconds or 0.0):.3f}s'
                )
                return

            resolved = self._resolve(command, state)
            self._command_id = self.backend.start_task_command(resolved)
            self._active_command = resolved
            self._command_deadline = now + self._command_timeout_seconds(
                resolved,
                state,
            )
            self.on_status(
                f'Task step {self._index + 1}/{len(self._task.commands)}: '
                f'{command.kind.value}'
            )
        except Exception as exc:
            self._fail(str(exc))

    def _tick_base(self, now: float) -> None:
        if self._base_velocity is None:
            raise RuntimeError('active base velocity is missing')
        if now < float(self._base_deadline):
            # Refresh faster than the control backend's cmd_vel stale watchdog.
            self.backend.set_velocity(*self._base_velocity)
            return
        self.backend.stop(publish_immediately=True)
        self._base_deadline = None
        self._base_velocity = None
        self._complete_step()
        self._request_stream_transition(False, now)

    def _request_stream_transition(self, enabled: bool, now: float) -> None:
        self.backend.request_stream(enabled)
        self._stream_target = enabled
        self._stream_deadline = now + 3.0

    def _complete_step(self) -> None:
        self._index += 1
        if self._task is not None:
            self.on_status(
                f'Task step {self._index}/{len(self._task.commands)} completed'
            )

    def _cancel_active_motion(self) -> None:
        command_cancel_requested = False
        if self._command_id is not None:
            try:
                self.backend.cancel_task_command(self._command_id)
                command_cancel_requested = True
            except Exception as exc:
                self.on_status(f'Task cancel warning: {exc}')
        if self._base_deadline is not None or self._stream_target is True:
            self.backend.stop(publish_immediately=True)
            try:
                self.backend.request_stream(False)
            except Exception as exc:
                self.on_status(f'Stream stop warning: {exc}')
        if not command_cancel_requested:
            self.backend.cancel_motion()

    def _finish(self) -> None:
        was_active = self.active
        self._timer.cancel()
        self._task = None
        self._index = 0
        self._command_id = None
        self._active_command = None
        self._command_deadline = 0.0
        self._delay_deadline = None
        self._base_deadline = None
        self._base_velocity = None
        self._stream_target = None
        self._stream_deadline = 0.0
        if was_active:
            self.on_active_changed(False)

    def _fail(self, message: str) -> None:
        name = self.task_name
        try:
            self._cancel_active_motion()
        except Exception as exc:
            self.on_status(f'Task cancel warning: {exc}')
        self._finish()
        self.on_status(f'Task failed ({name}): {message}')

    def _resolve(
        self,
        command: TaskCommand,
        state: TaskBackendState,
    ) -> TaskCommand:
        if command.kind is CommandKind.JOINT_RELATIVE:
            group = str(command.group)
            current = state.joint_groups.get(group)
            updated = state.joint_updated_at.get(group)
            if current is None or not state.joint_order_verified.get(group, False):
                raise RuntimeError('fresh, name-ordered joint state is required')
            self._require_fresh(updated, state.captured_at, 'joint state')
            return replace(
                command,
                kind=CommandKind.JOINT_ABSOLUTE,
                values=tuple(list_sum(current, command.values)),
            )
        if command.kind is CommandKind.LINEAR_RELATIVE:
            group = str(command.group)
            current = state.cartesian.get(group)
            updated = state.cartesian_updated_at.get(group)
            if current is None:
                raise RuntimeError('fresh Cartesian state is required')
            self._require_fresh(updated, state.captured_at, 'Cartesian state')
            return replace(
                command,
                kind=CommandKind.LINEAR_ABSOLUTE,
                values=tuple(list_sum(current, command.values)),
            )
        return command

    def _command_timeout_seconds(
        self,
        command: TaskCommand,
        state: TaskBackendState,
    ) -> float:
        expected = float(command.minimum_time or 0.0)
        if command.kind is CommandKind.JOINT_ABSOLUTE:
            current = self._fresh_values(
                state.joint_groups.get(str(command.group)),
                state.joint_updated_at.get(str(command.group)),
                state.captured_at,
                len(command.values),
            )
            expected = max(
                expected,
                self._joint_travel_seconds(
                    current,
                    command.values,
                    command.velocity_limit,
                ),
            )
        elif command.kind is CommandKind.JOINT_ABSOLUTE_MULTI:
            for group, target in command.joint_targets:
                current = self._fresh_values(
                    state.joint_groups.get(group),
                    state.joint_updated_at.get(group),
                    state.captured_at,
                    len(target),
                )
                expected = max(
                    expected,
                    self._joint_travel_seconds(
                        current,
                        target,
                        command.velocity_limit,
                    ),
                )
        elif command.kind is CommandKind.LINEAR_ABSOLUTE:
            current = self._fresh_values(
                state.cartesian.get(str(command.group)),
                state.cartesian_updated_at.get(str(command.group)),
                state.captured_at,
                len(command.values),
            )
            if current is not None:
                translation_distance = math.sqrt(sum(
                    (float(target) - float(start)) ** 2
                    for start, target in zip(current[:3], command.values[:3])
                ))
                rotation_distance = self._rotation_distance_rad(
                    current[3:6],
                    command.values[3:6],
                )
                expected = max(
                    expected,
                    translation_distance / float(command.linear_velocity),
                    rotation_distance / float(command.angular_velocity),
                )

        state_aware_timeout = (
            expected * self.MOTION_TIMEOUT_SCALE
            + self.MOTION_TIMEOUT_MARGIN_SEC
        )
        minimum_timeout = (
            self.CARTESIAN_TIMEOUT_MIN_SEC
            if command.kind is CommandKind.LINEAR_ABSOLUTE
            else self.MOTION_TIMEOUT_MIN_SEC
        )
        return max(
            minimum_timeout,
            command.timeout_seconds,
            state_aware_timeout,
        )

    def _timeout_detail(self, state: TaskBackendState) -> str:
        command = self._active_command
        if command is None or command.kind is not CommandKind.LINEAR_ABSOLUTE:
            return ''
        current = self._fresh_values(
            state.cartesian.get(str(command.group)),
            state.cartesian_updated_at.get(str(command.group)),
            state.captured_at,
            len(command.values),
        )
        if current is None:
            return '; current Cartesian pose is unavailable'
        position_error = math.sqrt(sum(
            (float(target) - float(start)) ** 2
            for start, target in zip(current[:3], command.values[:3])
        ))
        orientation_error = self._rotation_distance_rad(
            current[3:6],
            command.values[3:6],
        )
        return (
            f'; remaining position error={position_error:.6f} m, '
            f'orientation error={orientation_error:.6f} rad, '
            f'current TCP={[round(value, 6) for value in current]}, '
            f'target TCP={[round(value, 6) for value in command.values]}'
        )

    def _fresh_values(
        self,
        values,
        updated_at: Optional[float],
        captured_at: float,
        count: int,
    ):
        if values is None or len(values) != count or updated_at is None:
            return None
        age = captured_at - updated_at
        if not math.isfinite(age) or age < 0.0 or age > self.STATE_MAX_AGE_SEC:
            return None
        result = tuple(float(value) for value in values)
        if not all(math.isfinite(value) for value in result):
            return None
        return result

    @staticmethod
    def _joint_travel_seconds(current, target, velocity_limit) -> float:
        if current is None or velocity_limit is None:
            return 0.0
        max_distance_rad = max(
            math.radians(abs(float(goal) - float(start)))
            for start, goal in zip(current, target)
        )
        return max_distance_rad / float(velocity_limit)

    @staticmethod
    def _rotation_distance_rad(current_rpy, target_rpy) -> float:
        def quaternion(rpy):
            roll, pitch, yaw = (math.radians(float(value)) for value in rpy)
            cr, sr = math.cos(roll * 0.5), math.sin(roll * 0.5)
            cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
            cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
            return (
                sr * cp * cy - cr * sp * sy,
                cr * sp * cy + sr * cp * sy,
                cr * cp * sy - sr * sp * cy,
                cr * cp * cy + sr * sp * sy,
            )

        start_q = quaternion(current_rpy)
        target_q = quaternion(target_rpy)
        dot = abs(sum(a * b for a, b in zip(start_q, target_q)))
        return 2.0 * math.acos(max(0.0, min(1.0, dot)))

    def _require_safe_state(
        self,
        state: TaskBackendState,
        *,
        require_idle: bool,
    ) -> None:
        if not isinstance(state, TaskBackendState):
            raise RuntimeError('backend returned an invalid Task state')
        self._require_fresh(
            state.robot_state_updated_at,
            state.captured_at,
            'robot state',
        )
        if state.emo_active is not False:
            raise RuntimeError('EMO is active or unknown')
        if state.collision_active is not False:
            raise RuntimeError('collision state is active or unknown')
        if state.control_state not in (2, 3):
            raise RuntimeError('robot control state is not ENABLE/EXECUTING')
        if require_idle and state.motion_active:
            raise RuntimeError('another manipulation motion is active')

    @staticmethod
    def _require_fresh(
        updated_at: Optional[float],
        captured_at: float,
        label: str,
        maximum_age: float = STATE_MAX_AGE_SEC,
    ) -> None:
        if updated_at is None:
            raise RuntimeError(f'{label} is unavailable')
        age = captured_at - updated_at
        if not math.isfinite(age) or age < 0.0 or age > maximum_age:
            raise RuntimeError(f'{label} is stale')


TaskRunner = PlannerTaskRunner


__all__ = ['PlannerTaskRunner', 'TaskRunner']
