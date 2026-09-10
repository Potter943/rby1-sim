import pytest

from rby1_control.backend_contract import (
    TaskBackendState,
    TaskCommandState,
    TaskCommandStatus,
)
from rby1_planner.task_commands import Task
from rby1_planner.task_runner import PlannerTaskRunner


class FakeTimer:
    def __init__(self, callback):
        self.callback = callback
        self.active = True

    def cancel(self):
        self.active = False

    def reset(self):
        self.active = True


class FakeNode:
    def create_timer(self, _period_sec, callback):
        self.timer = FakeTimer(callback)
        return self.timer


class FakeBackend:
    def __init__(self):
        self.now = 100.0
        self.stream_enabled = False
        self.commands = []
        self.command_states = {}
        self.velocity_calls = []
        self.stop_calls = 0
        self.cancel_calls = 0
        self.stream_requests = []
        self.motion_active = False
        self.cancel_error = None

    def task_state(self):
        return TaskBackendState(
            captured_at=self.now,
            robot_state_updated_at=self.now - 0.1,
            control_state=2,
            stream_enabled=self.stream_enabled,
            emo_active=False,
            collision_active=False,
            motion_active=self.motion_active,
            joint_groups={'right_arm': (0.0,) * 7},
            joint_updated_at={'right_arm': self.now - 0.1},
            joint_order_verified={'right_arm': True},
            cartesian={'right_arm': (0.0,) * 6},
            cartesian_updated_at={'right_arm': self.now - 0.1},
            driver_safety_verified=True,
            driver_safety_updated_at=self.now - 0.1,
        )

    def start_task_command(self, command):
        command_id = f'command-{len(self.commands)}'
        self.commands.append(command)
        self.command_states[command_id] = TaskCommandState(
            TaskCommandStatus.SUCCEEDED,
            'done',
        )
        return command_id

    def poll_task_command(self, command_id):
        return self.command_states[command_id]

    def cancel_task_command(self, command_id):
        self.command_states[command_id] = TaskCommandState(
            TaskCommandStatus.CANCELED,
            'canceled',
        )

    def cancel_motion(self):
        self.cancel_calls += 1
        if self.cancel_error is not None:
            raise self.cancel_error

    def set_velocity(self, vx, vy, wz):
        self.velocity_calls.append((vx, vy, wz))

    def stop(self, publish_immediately=True):
        assert publish_immediately
        self.stop_calls += 1

    def request_stream(self, enabled):
        self.stream_requests.append(enabled)


def test_runner_uses_ros_timer_and_resolves_relative_joint_command():
    node = FakeNode()
    backend = FakeBackend()
    messages = []
    runner = PlannerTaskRunner(
        node,
        backend,
        on_status=messages.append,
        clock=lambda: backend.now,
    )
    assert not node.timer.active

    task = Task('relative').joint_relative(
        'right_arm',
        [1, 2, 3, 4, 5, 6, 7],
        [2.0, 0.2, 0.2],
    )
    runner.start(task.build())

    assert node.timer.active
    assert backend.commands[0].values == (1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0)
    runner.tick()
    assert not runner.active
    assert not node.timer.active
    assert messages[-1] == 'Task completed: relative'


def test_stop_while_waiting_for_base_stream_requests_stream_off():
    node = FakeNode()
    backend = FakeBackend()
    runner = PlannerTaskRunner(node, backend, clock=lambda: backend.now)
    task = Task('base').base_velocity(0.1, 0.0, 0.0, 1.0).build()

    runner.start(task)
    assert backend.stream_requests == [True]
    runner.stop()

    assert backend.stop_calls == 1
    assert backend.stream_requests == [True, False]
    assert not runner.active


def test_runner_delay_is_non_blocking():
    node = FakeNode()
    backend = FakeBackend()
    runner = PlannerTaskRunner(node, backend, clock=lambda: backend.now)
    runner.start(Task('wait').delay(100).build())

    assert runner.active
    backend.now += 0.05
    runner.tick()
    assert runner.active
    backend.now += 0.06
    runner.tick()
    assert not runner.active


def test_runner_rejects_stale_robot_state_before_start():
    node = FakeNode()
    backend = FakeBackend()
    backend.now = 10.0

    original_task_state = backend.task_state

    def stale_state():
        state = original_task_state()
        return TaskBackendState(
            **{
                **state.__dict__,
                'robot_state_updated_at': 8.0,
            }
        )

    backend.task_state = stale_state
    runner = PlannerTaskRunner(node, backend, clock=lambda: backend.now)

    with pytest.raises(RuntimeError, match='robot state is stale'):
        runner.start(Task('wait').delay(100).build())


def test_base_velocity_is_refreshed_and_stopped():
    node = FakeNode()
    backend = FakeBackend()
    backend.stream_enabled = True
    runner = PlannerTaskRunner(node, backend, clock=lambda: backend.now)
    runner.start(Task('base').base_velocity(0.1, 0.0, 0.0, 0.1).build())

    backend.now += 0.05
    runner.tick()
    assert len(backend.velocity_calls) == 2

    backend.now += 0.06
    runner.tick()
    assert backend.stop_calls == 1
    assert backend.stream_requests == [False]

    backend.stream_enabled = False
    runner.tick()
    assert not runner.active


def test_base_motion_fails_safe_if_manipulation_becomes_active():
    node = FakeNode()
    backend = FakeBackend()
    backend.stream_enabled = True
    messages = []
    runner = PlannerTaskRunner(
        node,
        backend,
        on_status=messages.append,
        clock=lambda: backend.now,
    )
    runner.start(Task('base').base_velocity(0.1, 0.0, 0.0, 1.0).build())

    backend.motion_active = True
    backend.now += 0.05
    runner.tick()

    assert not runner.active
    assert backend.stop_calls == 1
    assert backend.stream_requests[-1] is False
    assert messages[-1].startswith('Task failed (base):')


def test_stop_finishes_runner_when_cancellation_raises():
    node = FakeNode()
    backend = FakeBackend()
    backend.cancel_error = RuntimeError('cancel transport unavailable')
    messages = []
    runner = PlannerTaskRunner(
        node,
        backend,
        on_status=messages.append,
        clock=lambda: backend.now,
    )
    runner.start(Task('delay').delay(1.0).build())

    runner.stop()

    assert not runner.active
    assert not node.timer.active
    assert 'Task cancel warning: cancel transport unavailable' in messages
    assert messages[-1] == 'Task stopped'
