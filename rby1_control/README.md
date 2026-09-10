# RB-Y1 Control

`rby1_control` owns the robot-facing control backend. It also keeps the former
Qt frontend and IDE-authored Scenario UI as a command-validation and debugging
tool. Production launch does not start the Qt UI.

Camera and perception logic intentionally do not belong to this package. The
planner consumes object poses and sends validated commands through the control
topics below.

## Nodes and topics

The backend is `/rby1/rby1_control_backend`. A frontend or planner communicates
with it through a versioned JSON envelope carried by `std_msgs/msg/String`:

```text
frontend or planner
  -> /rby1/control/command
  -> /rby1/rby1_control_backend
  -> RB-Y1 driver topics, services, and actions

/rby1/rby1_control_backend
  -> /rby1/control/state
  -> /rby1/control/event
  -> /rby1/control/response
  -> frontend or planner
```

`topic_protocol.py` defines schema version 1. Supported operations are
`set_velocity`, `stop`, `prepare_robot`, `request_power`, `request_servo`,
`request_stream`, `request_control_manager`, `request_cartesian_snapshot`,
`jog_joint`, `move_joint_group`, `jog_cartesian`, `move_cartesian`,
`cancel_active_motion`, `cancel_motion`, `start_task_command`,
`cancel_task_command`, and `shutdown_safely`.

Robot-facing validation remains in the backend. Publishing a transport command
does not bypass state freshness, joint-limit, collision, or action-conflict
checks. A successful `control/response` means the request was accepted; motion
completion is reported later in `control/state` and `control/event`.

## Launch

Run the production backend without a GUI:

```bash
ros2 launch rby1_control control.launch.py
```

Run the backend together with the Qt command-validation UI:

```bash
ros2 launch rby1_control control_debug.launch.py
```

Both launches accept `namespace` and `config` arguments. The two executables can
also be started separately:

```bash
ros2 run rby1_control control_backend --ros-args --params-file <config.yaml>
ros2 run rby1_control control_ui --ros-args --params-file <config.yaml>
```

Do not run the Qt debug frontend while an autonomous planner is commanding the
same backend. The debug frontend continuously refreshes base commands, including
zero velocity while idle, and the transport currently has no command-owner
arbitration.

There is no mock backend or `--mock` mode in this package.

## Debug UI and Scenario Tasks

The copied Qt frontend retains manual mobile-base, joint, and Cartesian
controls, diagnostics, and the Scenario tab. Edit
`rby1_control/rby1_control/task.py`, build with `--symlink-install`, and use
**Reload Tasks** in the Scenario tab.

The validated Task catalog is written atomically to
`${XDG_CONFIG_HOME:-~/.config}/rby1_control/task_catalog.json`. Set
`RBY1_TASK_CATALOG` to override the path.

Task units are:

- joint positions: degrees;
- Cartesian position: metres;
- Cartesian orientation: roll/pitch/yaw degrees in the base frame;
- joint motion settings: minimum time (s), velocity (rad/s), acceleration
  (rad/s²);
- Cartesian motion settings: minimum time (s), linear velocity (m/s), angular
  velocity (rad/s), acceleration scaling (0.0-1.0).

`joint_absolute_multi()` sends multiple body groups in one synchronized action
goal. `whole_body_joint_absolute()` is the torso-plus-both-arms convenience
form. Task execution remains non-blocking and checks robot state, EMO/collision
state, command timeout, and driver results for every step.

## Reusable frontend API

Headless clients can reuse the same topic proxy as the debug UI:

```python
from rby1_control.frontend_node import ControlFrontendNode

node = ControlFrontendNode(node_name='rby1_planner', namespace='rby1')
```

`ControlUiFrontendNode` remains as a compatibility alias. Task and wire types
are available from `rby1_control.task_commands`,
`rby1_control.backend_contract`, and `rby1_control.topic_protocol` so clients do
not need to duplicate the protocol implementation.

## Qt dependency

The debug UI supports PySide6 or PyQt5. Only one binding is required. The
production backend launch does not import Qt.

## Package layout

```text
rby1_control/
├── package.xml
├── setup.py
├── setup.cfg
├── resource/rby1_control
├── config/default.yaml
├── launch/
│   ├── control.launch.py
│   └── control_debug.launch.py
└── rby1_control/
    ├── __init__.py
    ├── backend_contract.py
    ├── backend_main.py
    ├── frontend_node.py
    ├── main.py
    ├── main_window.py
    ├── qt_compat.py
    ├── ros_backend.py
    ├── scenario_ui.py
    ├── task.py
    ├── task_commands.py
    ├── task_runner.py
    ├── topic_backend.py
    └── topic_protocol.py
```
