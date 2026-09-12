# RB-Y1 Planner

`rby1_planner` is the headless automation frontend for `rby1_control`. It
subscribes to object observations, resolves them through TF, authors the same
validated Task commands used by the debug UI, and sends those commands through
the production control topics.

Starting this package is safe by default: the node only creates its
publishers, subscriptions, TF listener, and an inactive ROS timer. It does not
prepare the robot, enable streaming, or run a Task until application code
explicitly calls `PlannerNode.run_task()` or another inherited command method.
The headless entry point has no Qt dependency. The optional operator UI uses
PyQt5 (or PySide6 when available); there is no runtime mock implementation in
this package.

## Architecture

```text
camera / detector
  -> /rby1/perception/object_pose (apriltag_msgs/AprilTagDetectionArray)
  -> CameraClient + PlannerNode
  -> /rby1/control/command        (std_msgs/String, protocol v1)
  -> rby1_control backend

rby1_control backend
  -> /rby1/control/state
  -> /rby1/control/event
  -> /rby1/control/response
  -> PlannerNode
```

`PlannerNode` subclasses `rby1_control.frontend_node.ControlFrontendNode`, so
the planner and debug UI use one implementation of connection detection,
request IDs, state caching, command serialization, response filtering, and
Task status polling. `rby1_planner.task_commands` re-exports the canonical
immutable command types and extends the `Task` builder with planner-only
camera steps; the control wire schema is not duplicated.

The two `task.py` modules have deliberately different jobs. The control
package keeps the copied, user-editable static Scenario catalog for the debug
UI. `rby1_planner.task` instead contains a runtime factory that composes a
validated camera observation with a caller-supplied object-to-end-effector
offset. Quaternion-to-RPY conversion uses the same convention and degree units
as the control backend.

The inherited command API includes `set_velocity`, `stop`, `prepare_robot`,
power/servo/stream/control-manager requests, joint and Cartesian moves,
motion cancellation, `start_task_command`, and Task status polling.
`PlannerTaskRunner` executes ordinary canonical Tasks and planner-only dynamic
Tasks sequentially with a 50 ms ROS timer. A dynamic camera step is resolved
to a canonical `LINEAR_ABSOLUTE` command inside the planner before crossing the
control backend boundary. The runner retains the UI runner's fresh-state, EMO,
collision, control-state, command-result, and distance-aware timeout checks
without using a Qt event loop.

## Camera contract

`CameraClient` subscribes with best-effort, volatile, depth-5 sensor semantics.
Each AprilTag detection must have:

- a valid nonnegative tag ID, family, and `header.frame_id`;
- a valid ROS timestamp;
- a matching TF from the requested target frame to the tag frame.

It caches the newest valid detection per tag, ignores out-of-order samples,
rejects stale/future-dated data at query time, and uses
`tf2_ros.Buffer.lookup_transform()` at the original detection timestamp. The
returned `ObjectObservation` is deeply immutable: position and quaternion are
tuples rather than mutable ROS message fields.

The camera pipeline is streaming; there is currently no hardware shutter or
capture service in the interface. To require a newly processed image, record a
marker and query until a later observation is available:

```python
marker_ns = node.camera.capture_marker()

# Call from a later ROS timer tick; do not block the executor.
observation = node.camera.get_observation(
    'tag_0',
    newer_than_ns=marker_ns,
)
if observation is not None:
    print(observation.position, observation.orientation_xyzw)
```

With the default target frame, the result is `base <- object`. A grasp planner
can compose that transform with its desired `object <- end_effector` offset and
author a Cartesian Task. CameraClient deliberately does not turn detections
directly into robot motion.

## Building and running a dynamic camera Task

```python
from rby1_planner.task_commands import Task

task = Task('detect-between-motions')
task.extend(first_motion())
task.camera_linear_absolute(
    arm='right_arm',
    object_id='tag_0',
    tcp_motion=[1.0, 0.05, 0.2, 0.5],
    detection_timeout_sec=3.0,
    object_to_end_effector_position=calibrated_object_to_ee_xyz,
    object_to_end_effector_orientation_xyzw=calibrated_object_to_ee_quaternion,
)
task.extend(last_motion())
node.run_task(task.build())
```

The runner finishes `first_motion`, records a camera capture marker, waits
without blocking the ROS executor for a later detection and matching TF,
converts its `base` pose into a canonical Cartesian command, and only then
runs `last_motion`. A detection timeout fails the Task without sending a
placeholder Cartesian target.

If the object-to-end-effector transform is omitted, its identity default sends
the TCP to the tag pose itself. The registered mock demo instead stops 10 cm
above the upward-facing tag and retains its measured downward-facing pickup
orientation. Real tasks must supply a calibrated approach or grasp transform.

Dynamic Tasks are planner-local runtime objects; the control debug UI catalog
cannot serialize them. Merely launching `rby1_planner` never runs a Task.

## Run

Build `rby1_control`, `rby1_msgs`, and this package in the same workspace. Start
the production backend first, then choose either the headless planner or its
Qt operator UI:

```bash
ros2 launch rby1_control control.launch.py

# Headless planner
ros2 launch rby1_planner planner.launch.py

# Or planner UI; do not launch this together with the headless planner.
ros2 launch rby1_planner planner_ui.launch.py
```

The planner launches do not start a camera driver, perception producer, or
control backend. Those lifecycle choices stay explicit. The planner UI reuses
the control UI's Base, Joints and Diagnostics tabs, and replaces the Scenario
tab with a direct loader for planner `task.py`. The current Task List registers
`move_right_ee_to_detected_object`, `object_handover_demo2`, and
`object_gripping_initial_pose`.

The control debug UI and planner publish to the same command topic. Until the
backend has a controller-ownership/arbitration policy, do not issue motion from
both frontends at the same time. Also do not launch the old
`rby1_control_ui` backend alongside `rby1_control`; two subscribers on the same
command topic would execute each command twice.

## Integration prerequisite

The hardware bringup must publish the robot TF to the end effector, and the
camera launch must publish the calibrated static end-effector-to-camera mount
transform. Without the complete TF chain from `base` to the detected tag,
CameraClient safely returns no Cartesian target and a dynamic step eventually
times out.

## Parameters

The defaults are in `config/default.yaml`:

- `object_pose_topic`: `perception/object_pose`
- `object_pose_target_frame`: `base`
- `object_pose_max_age_sec`: `0.5`
- `object_pose_min_confidence`: `0.5`
- `object_pose_future_tolerance_sec`: `0.05`
- `object_pose_tf_timeout_sec`: `0.0`

The TF timeout must remain zero for the default single-threaded executor: a
blocking lookup could prevent that same executor from receiving TF. The
constructor rejects a positive value. Planner logic should retry on later
timer ticks when `get_observation()` returns `None`.
