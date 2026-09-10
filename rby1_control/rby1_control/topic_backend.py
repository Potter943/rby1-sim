"""Robot-facing backend node exposed through versioned ROS topics."""
from __future__ import annotations

import math
from typing import Dict

from std_msgs.msg import String

from .backend_contract import TaskCommandState, TaskCommandStatus
from .ros_backend import Rby1ControlNode
from .topic_protocol import (
    COMMAND_KIND,
    EVENT_KIND,
    RESPONSE_KIND,
    STATE_KIND,
    backend_snapshot_to_dict,
    decode_message,
    encode_message,
    task_backend_state_to_dict,
    task_command_from_dict,
    task_command_state_to_dict,
)


class TopicControlBackendNode(Rby1ControlNode):
    """Own robot interfaces and execute commands received from any publisher."""

    def __init__(self) -> None:
        super().__init__(node_name='rby1_control_backend')

        self.declare_parameter('control_command_topic', 'control/command')
        self.declare_parameter('control_state_topic', 'control/state')
        self.declare_parameter('control_event_topic', 'control/event')
        self.declare_parameter('control_response_topic', 'control/response')
        self.declare_parameter('control_state_publish_rate_hz', 20.0)

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
        state_rate_hz = self._positive_float(
            self.get_parameter('control_state_publish_rate_hz').value,
            fallback=20.0,
        )

        self.transport_state_pub = self.create_publisher(
            String,
            self.control_state_topic,
            10,
        )
        self.transport_event_pub = self.create_publisher(
            String,
            self.control_event_topic,
            50,
        )
        self.transport_response_pub = self.create_publisher(
            String,
            self.control_response_topic,
            50,
        )
        self.transport_command_sub = self.create_subscription(
            String,
            self.control_command_topic,
            self._transport_command_callback,
            50,
        )

        self._transport_task_routes: Dict[str, str] = {}
        self._transport_task_states: Dict[str, TaskCommandState] = {}
        self.transport_state_timer = self.create_timer(
            1.0 / state_rate_hz,
            self._publish_transport_state,
        )

        self._push_event(
            'info',
            'Topic command interface ready: '
            f'{self.resolve_topic_name(self.control_command_topic)}',
        )

    def _transport_command_callback(self, message: String) -> None:
        operation = '<invalid>'
        source = ''
        request_id = ''
        try:
            packet = decode_message(message.data, expected_kind=COMMAND_KIND)
            operation = str(packet.get('operation', ''))
            source = str(packet.get('source', ''))
            request_id = str(packet.get('request_id', ''))
            arguments = packet.get('arguments', {})
            if not operation:
                raise ValueError('command operation is missing')
            if not isinstance(arguments, dict):
                raise ValueError('command arguments must be an object')

            detail = self._dispatch_transport_command(
                operation,
                arguments,
                request_id=request_id,
            )
            if request_id:
                self._publish_response(
                    source=source,
                    request_id=request_id,
                    operation=operation,
                    accepted=True,
                    message=detail,
                )
        except Exception as exc:
            detail = str(exc)
            if operation == 'start_task_command' and request_id:
                self._transport_task_states[request_id] = TaskCommandState(
                    TaskCommandStatus.FAILED,
                    detail,
                )
            self._push_event(
                'error',
                f'Topic command {operation!r} rejected: {detail}',
            )
            if request_id:
                self._publish_response(
                    source=source,
                    request_id=request_id,
                    operation=operation,
                    accepted=False,
                    message=detail,
                )

    def _dispatch_transport_command(
        self,
        operation: str,
        arguments: dict,
        *,
        request_id: str,
    ) -> str:
        if operation == 'set_velocity':
            self.set_velocity(
                self._finite_argument(arguments, 'vx', default=0.0),
                self._finite_argument(arguments, 'vy', default=0.0),
                self._finite_argument(arguments, 'wz', default=0.0),
            )
        elif operation == 'stop':
            self.stop(
                publish_immediately=self._bool_argument(
                    arguments,
                    'publish_immediately',
                    default=True,
                )
            )
        elif operation == 'prepare_robot':
            self.prepare_robot()
        elif operation == 'request_power':
            self.request_power(
                self._bool_argument(arguments, 'enabled'),
                target=str(arguments.get('target', 'all')),
            )
        elif operation == 'request_servo':
            self.request_servo(
                self._bool_argument(arguments, 'enabled'),
                target=str(arguments.get('target', 'all')),
            )
        elif operation == 'request_stream':
            self.request_stream(
                self._bool_argument(arguments, 'enabled'),
                value=self._finite_argument(arguments, 'value', default=0.0),
            )
        elif operation == 'request_control_manager':
            self.request_control_manager(str(arguments.get('command', '')))
        elif operation == 'request_cartesian_snapshot':
            if not self.request_cartesian_snapshot(str(arguments.get('arm', ''))):
                raise RuntimeError('Cartesian snapshot request could not start')
        elif operation == 'jog_joint':
            self.jog_joint(
                str(arguments.get('group', '')),
                int(arguments.get('joint_index', -1)),
                float(arguments.get('delta_deg', 0.0)),
            )
        elif operation == 'move_joint_group':
            self.move_joint_group(
                str(arguments.get('group', '')),
                list(arguments.get('targets_deg', ())),
                float(arguments.get('minimum_time', 3.0)),
            )
        elif operation == 'jog_cartesian':
            self.jog_cartesian(
                str(arguments.get('arm', '')),
                int(arguments.get('axis_index', -1)),
                float(arguments.get('delta', 0.0)),
                reference_frame=str(arguments.get('reference_frame', 'base')),
            )
        elif operation == 'move_cartesian':
            self.move_cartesian(
                str(arguments.get('arm', '')),
                list(arguments.get('target', ())),
                float(arguments.get('minimum_time', 3.0)),
                reference_frame=str(arguments.get('reference_frame', 'base')),
            )
        elif operation == 'cancel_active_motion':
            self.cancel_active_motion()
        elif operation == 'cancel_motion':
            self.cancel_motion()
        elif operation == 'start_task_command':
            if not request_id:
                raise ValueError('start_task_command requires request_id')
            if request_id in self._transport_task_routes:
                raise ValueError(f'duplicate Task request_id: {request_id}')
            command = task_command_from_dict(arguments.get('command', {}))
            backend_id = self.start_task_command(command)
            self._transport_task_routes[request_id] = backend_id
            self._transport_task_states[request_id] = TaskCommandState(
                TaskCommandStatus.PENDING,
                'Backend accepted command',
            )
        elif operation == 'cancel_task_command':
            external_id = str(arguments.get('command_id', ''))
            backend_id = self._transport_task_routes.get(external_id)
            if backend_id is None:
                raise KeyError(f'unknown Task command: {external_id}')
            self.cancel_task_command(backend_id)
        elif operation == 'shutdown_safely':
            self.shutdown_safely(
                turn_stream_off=self._bool_argument(
                    arguments,
                    'turn_stream_off',
                    default=True,
                )
            )
        else:
            raise ValueError(f'unsupported command operation: {operation}')
        return 'accepted'

    @staticmethod
    def _bool_argument(
        arguments: dict,
        name: str,
        *,
        default=None,
    ) -> bool:
        value = arguments.get(name, default)
        if not isinstance(value, bool):
            raise ValueError(f'{name} must be a JSON boolean')
        return value

    @staticmethod
    def _finite_argument(
        arguments: dict,
        name: str,
        *,
        default=None,
    ) -> float:
        try:
            value = float(arguments.get(name, default))
        except (TypeError, ValueError) as exc:
            raise ValueError(f'{name} must be a finite number') from exc
        if not math.isfinite(value):
            raise ValueError(f'{name} must be a finite number')
        return value

    def _publish_response(
        self,
        *,
        source: str,
        request_id: str,
        operation: str,
        accepted: bool,
        message: str,
    ) -> None:
        response = String()
        response.data = encode_message(
            RESPONSE_KIND,
            {
                'source': source,
                'request_id': request_id,
                'operation': operation,
                'accepted': bool(accepted),
                'message': str(message),
            },
        )
        self.transport_response_pub.publish(response)

    def _refresh_transport_task_states(self) -> None:
        for external_id, backend_id in tuple(self._transport_task_routes.items()):
            try:
                self._transport_task_states[external_id] = self.poll_task_command(
                    backend_id
                )
            except Exception as exc:
                self._transport_task_states[external_id] = TaskCommandState(
                    TaskCommandStatus.FAILED,
                    str(exc),
                )

    def _publish_transport_state(self) -> None:
        try:
            self._refresh_transport_task_states()
            packet = {
                'snapshot': backend_snapshot_to_dict(self.snapshot()),
                'task_state': task_backend_state_to_dict(self.task_state()),
                'motion_state': self.get_motion_state(),
                'cartesian_snapshots': {
                    arm: self.get_cartesian_snapshot(arm)
                    for arm in ('right_arm', 'left_arm')
                },
                'task_commands': {
                    command_id: task_command_state_to_dict(state)
                    for command_id, state in self._transport_task_states.items()
                },
            }
            state_message = String()
            state_message.data = encode_message(STATE_KIND, packet)
            self.transport_state_pub.publish(state_message)
        except Exception as exc:
            self._push_event('error', f'Failed to publish backend state: {exc}')

        for level, text in self.drain_events():
            event_message = String()
            event_message.data = encode_message(
                EVENT_KIND,
                {'level': str(level), 'message': str(text)},
            )
            self.transport_event_pub.publish(event_message)
