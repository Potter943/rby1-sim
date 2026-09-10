"""Frontend ROS node and backend-compatible control-topic proxy."""
from __future__ import annotations

from collections import deque
from dataclasses import replace
import threading
import time
from typing import Deque, Dict, List, Optional, Tuple
import uuid

from rclpy.node import Node
from std_msgs.msg import String

from .backend_contract import (
    BackendSnapshot,
    TaskBackendState,
    TaskCommandState,
    TaskCommandStatus,
    VelocityCommand,
)
from .task_commands import TaskCommand
from .topic_protocol import (
    COMMAND_KIND,
    EVENT_KIND,
    RESPONSE_KIND,
    STATE_KIND,
    backend_snapshot_from_dict,
    decode_message,
    encode_message,
    task_backend_state_from_dict,
    task_command_state_from_dict,
    task_command_to_dict,
)


class ControlFrontendNode(Node):
    """ROS node that talks only to the backend transport topics.

    The Qt debug UI uses the compatibility alias below.  Headless clients such
    as ``rby1_planner`` can provide their own node name while reusing the exact
    same command, state, event, and response protocol implementation.
    """

    backend_name = 'ROS2 TOPIC'

    def __init__(
        self,
        node_name: str = 'rby1_control_debug_ui',
        namespace: Optional[str] = 'rby1',
    ) -> None:
        super().__init__(str(node_name), namespace=namespace)

        self.declare_parameter('control_command_topic', 'control/command')
        self.declare_parameter('control_state_topic', 'control/state')
        self.declare_parameter('control_event_topic', 'control/event')
        self.declare_parameter('control_response_topic', 'control/response')
        self.declare_parameter('control_state_timeout_sec', 1.0)

        self.control_command_topic = str(
            self.get_parameter('control_command_topic').value
        )
        self.control_state_topic = str(
            self.get_parameter('control_state_topic').value
        )
        self.control_event_topic = str(
            self.get_parameter('control_event_topic').value
        )
        self.control_response_topic = str(
            self.get_parameter('control_response_topic').value
        )
        timeout = float(self.get_parameter('control_state_timeout_sec').value)
        self.control_state_timeout_sec = timeout if timeout > 0.0 else 1.0

        self.command_pub = self.create_publisher(
            String,
            self.control_command_topic,
            50,
        )
        self.state_sub = self.create_subscription(
            String,
            self.control_state_topic,
            self._state_callback,
            10,
        )
        self.event_sub = self.create_subscription(
            String,
            self.control_event_topic,
            self._event_callback,
            50,
        )
        self.response_sub = self.create_subscription(
            String,
            self.control_response_topic,
            self._response_callback,
            50,
        )

        self._lock = threading.RLock()
        self._source_id = f'{self.get_fully_qualified_name()}:{uuid.uuid4().hex}'
        self._last_state_received_at: Optional[float] = None
        self._snapshot: Optional[BackendSnapshot] = None
        self._task_state: Optional[TaskBackendState] = None
        self._motion_state: Dict[str, object] = {
            'joint_groups': {},
            'cartesian': {},
        }
        self._cartesian_snapshots: Dict[str, Optional[List[float]]] = {
            'right_arm': None,
            'left_arm': None,
        }
        self._task_commands: Dict[str, TaskCommandState] = {}
        self._events: Deque[Tuple[str, str]] = deque(maxlen=300)

        self._push_local_event(
            'info',
            'Frontend node started; waiting for backend state on '
            f'{self.resolve_topic_name(self.control_state_topic)}.',
        )

    def _connected(self) -> bool:
        with self._lock:
            updated_at = self._last_state_received_at
        return (
            updated_at is not None
            and time.monotonic() - updated_at <= self.control_state_timeout_sec
        )

    def _send(
        self,
        operation: str,
        arguments: Optional[dict] = None,
        *,
        request_id: str = '',
    ) -> None:
        message = String()
        message.data = encode_message(
            COMMAND_KIND,
            {
                'source': self._source_id,
                'request_id': str(request_id),
                'operation': str(operation),
                'arguments': dict(arguments or {}),
            },
        )
        self.command_pub.publish(message)

    def _state_callback(self, message: String) -> None:
        try:
            packet = decode_message(message.data, expected_kind=STATE_KIND)
            snapshot = backend_snapshot_from_dict(packet.get('snapshot', {}))
            task_state = task_backend_state_from_dict(
                packet.get('task_state', {})
            )
            motion_state = packet.get('motion_state', {})
            cartesian_snapshots = packet.get('cartesian_snapshots', {})
            raw_task_commands = packet.get('task_commands', {})
            if not isinstance(motion_state, dict):
                raise ValueError('backend motion_state must be an object')
            if not isinstance(cartesian_snapshots, dict):
                raise ValueError('backend Cartesian snapshots must be an object')
            if not isinstance(raw_task_commands, dict):
                raise ValueError('backend Task states must be an object')

            task_commands = {
                str(command_id): task_command_state_from_dict(state)
                for command_id, state in raw_task_commands.items()
            }
            normalized_snapshots = {
                str(arm): (
                    [float(value) for value in values]
                    if values is not None
                    else None
                )
                for arm, values in cartesian_snapshots.items()
            }

            with self._lock:
                self._snapshot = snapshot
                self._task_state = task_state
                self._motion_state = motion_state
                self._cartesian_snapshots.update(normalized_snapshots)
                self._task_commands.update(task_commands)
                self._last_state_received_at = time.monotonic()
        except Exception as exc:
            self._push_local_event(
                'error',
                f'Invalid backend state payload: {exc}',
            )

    def _event_callback(self, message: String) -> None:
        try:
            packet = decode_message(message.data, expected_kind=EVENT_KIND)
            self._append_event(
                str(packet.get('level', 'info')),
                str(packet.get('message', '')),
            )
        except Exception as exc:
            self._push_local_event('error', f'Invalid backend event: {exc}')

    def _response_callback(self, message: String) -> None:
        try:
            packet = decode_message(message.data, expected_kind=RESPONSE_KIND)
            if str(packet.get('source', '')) != self._source_id:
                return
            if bool(packet.get('accepted', False)):
                return

            request_id = str(packet.get('request_id', ''))
            operation = str(packet.get('operation', ''))
            detail = str(packet.get('message', 'command rejected'))
            if operation == 'start_task_command' and request_id:
                with self._lock:
                    self._task_commands[request_id] = TaskCommandState(
                        TaskCommandStatus.FAILED,
                        detail,
                    )
        except Exception as exc:
            self._push_local_event('error', f'Invalid backend response: {exc}')

    def _append_event(self, level: str, message: str) -> None:
        with self._lock:
            self._events.append((str(level), str(message)))

    def _push_local_event(self, level: str, message: str) -> None:
        stamp = time.strftime('%H:%M:%S')
        self._append_event(level, f'[{stamp}] {message}')

    def drain_events(self) -> List[Tuple[str, str]]:
        with self._lock:
            events = list(self._events)
            self._events.clear()
        return events

    def snapshot(self) -> BackendSnapshot:
        with self._lock:
            snapshot = self._snapshot
        if snapshot is not None and self._connected():
            return snapshot

        command = snapshot.command if snapshot is not None else VelocityCommand()
        return BackendSnapshot(
            namespace=self.get_namespace(),
            cmd_vel_topic=(snapshot.cmd_vel_topic if snapshot is not None else ''),
            cmd_vel_subscribers=0,
            control_state=None,
            power_enabled=None,
            servo_enabled=None,
            stream_enabled=None,
            emo_active=None,
            collision_active=None,
            services_enabled=False,
            rby1_msgs_available=False,
            service_ready={},
            command=command,
            command_stale=True,
        )

    def task_state(self) -> TaskBackendState:
        now = time.monotonic()
        with self._lock:
            state = self._task_state
        if state is not None and self._connected():
            # Backend timestamps and time.monotonic() share the same host clock.
            # Capture at read time so transport delay is included in freshness.
            return replace(state, captured_at=now)
        return TaskBackendState(
            captured_at=now,
            robot_state_updated_at=None,
            control_state=None,
            stream_enabled=None,
            emo_active=None,
            collision_active=None,
            motion_active=False,
            joint_groups={},
            joint_updated_at={},
            joint_order_verified={},
            cartesian={},
            cartesian_updated_at={},
            driver_safety_verified=False,
            driver_safety_updated_at=None,
        )

    def get_motion_state(self) -> dict:
        with self._lock:
            return dict(self._motion_state)

    def get_joint_snapshot(self, group: str) -> Optional[List[float]]:
        with self._lock:
            values = self._motion_state.get('joint_groups', {}).get(group)
        return list(values) if values is not None else None

    def request_cartesian_snapshot(self, arm: str) -> bool:
        if not self._connected():
            return False
        with self._lock:
            self._cartesian_snapshots[str(arm)] = None
        self._send('request_cartesian_snapshot', {'arm': str(arm)})
        return True

    def get_cartesian_snapshot(self, arm: str) -> Optional[List[float]]:
        with self._lock:
            values = self._cartesian_snapshots.get(str(arm))
        return list(values) if values is not None else None

    def set_velocity(self, vx: float, vy: float, wz: float) -> None:
        self._send(
            'set_velocity',
            {'vx': float(vx), 'vy': float(vy), 'wz': float(wz)},
        )

    def stop(self, publish_immediately: bool = True) -> None:
        self._send(
            'stop',
            {'publish_immediately': bool(publish_immediately)},
        )

    def prepare_robot(self) -> None:
        self._send('prepare_robot')

    def request_power(self, enabled: bool, target: str = 'all') -> None:
        self._send(
            'request_power',
            {'enabled': bool(enabled), 'target': str(target)},
        )

    def request_servo(self, enabled: bool, target: str = 'all') -> None:
        self._send(
            'request_servo',
            {'enabled': bool(enabled), 'target': str(target)},
        )

    def request_stream(self, enabled: bool, value: float = 0.0) -> None:
        self._send(
            'request_stream',
            {'enabled': bool(enabled), 'value': float(value)},
        )

    def request_control_manager(self, command: str) -> None:
        self._send('request_control_manager', {'command': str(command)})

    def jog_joint(self, group: str, joint_index: int, delta_deg: float) -> None:
        self._send(
            'jog_joint',
            {
                'group': str(group),
                'joint_index': int(joint_index),
                'delta_deg': float(delta_deg),
            },
        )

    def move_joint_group(
        self,
        group: str,
        targets_deg,
        minimum_time: float,
    ) -> None:
        self._send(
            'move_joint_group',
            {
                'group': str(group),
                'targets_deg': [float(value) for value in targets_deg],
                'minimum_time': float(minimum_time),
            },
        )

    def jog_cartesian(
        self,
        arm: str,
        axis_index: int,
        delta: float,
        reference_frame: str = 'base',
    ) -> None:
        self._send(
            'jog_cartesian',
            {
                'arm': str(arm),
                'axis_index': int(axis_index),
                'delta': float(delta),
                'reference_frame': str(reference_frame),
            },
        )

    def move_cartesian(
        self,
        arm: str,
        target,
        minimum_time: float,
        reference_frame: str = 'base',
    ) -> None:
        self._send(
            'move_cartesian',
            {
                'arm': str(arm),
                'target': [float(value) for value in target],
                'minimum_time': float(minimum_time),
                'reference_frame': str(reference_frame),
            },
        )

    def is_motion_busy(self) -> bool:
        return bool(self.task_state().motion_active)

    def cancel_active_motion(self) -> None:
        self._send('cancel_active_motion')

    def cancel_motion(self) -> None:
        self._send('cancel_motion')

    def start_task_command(self, command: TaskCommand) -> str:
        if not self._connected():
            raise RuntimeError('control backend state is unavailable')
        command_id = f'{self.get_name()}-{uuid.uuid4().hex}'
        with self._lock:
            self._task_commands[command_id] = TaskCommandState(
                TaskCommandStatus.PENDING,
                'Published to backend',
            )
        self._send(
            'start_task_command',
            {'command': task_command_to_dict(command)},
            request_id=command_id,
        )
        return command_id

    def poll_task_command(self, command_id: str) -> TaskCommandState:
        with self._lock:
            state = self._task_commands.get(str(command_id))
        if state is None:
            raise KeyError(f'unknown Task command: {command_id}')
        return state

    def cancel_task_command(self, command_id: str) -> None:
        self._send(
            'cancel_task_command',
            {'command_id': str(command_id)},
        )
        with self._lock:
            if command_id in self._task_commands:
                self._task_commands[command_id] = TaskCommandState(
                    TaskCommandStatus.CANCELED,
                    'Cancel published to backend',
                )

    def run_diagnostics(self) -> dict:
        snapshot = self.snapshot()
        state = self.get_motion_state()
        return {
            'backend': self.backend_name,
            'backend_connected': self._connected(),
            'namespace': snapshot.namespace,
            'command_topic': self.resolve_topic_name(self.control_command_topic),
            'state_topic': self.resolve_topic_name(self.control_state_topic),
            'command_subscribers': self.command_pub.get_subscription_count(),
            'cmd_vel_topic': snapshot.cmd_vel_topic,
            'cmd_vel_subscribers': snapshot.cmd_vel_subscribers,
            'control_state': snapshot.control_state,
            'stream_enabled': snapshot.stream_enabled,
            'emo_active': snapshot.emo_active,
            'collision_active': snapshot.collision_active,
            'services_enabled': snapshot.services_enabled,
            'service_ready': dict(snapshot.service_ready),
            'joint_groups': state.get('joint_groups', {}),
            'cartesian': state.get('cartesian', {}),
        }

    def shutdown_safely(self, turn_stream_off: bool = True) -> None:
        self._send(
            'shutdown_safely',
            {'turn_stream_off': bool(turn_stream_off)},
        )


# Preserve the name used by the copied Qt frontend while exposing a generic
# name for planner and other non-UI clients.
ControlUiFrontendNode = ControlFrontendNode


__all__ = ['ControlFrontendNode', 'ControlUiFrontendNode']
