# RB-Y1 Planner

`rby1_planner` is the headless automation frontend for `rby1_control`. It
subscribes to object observations, resolves them through TF, authors the same
validated Task commands used by the debug UI, and sends those commands through
the production control topics.

Starting this package is safe by default: the node only creates its
publishers, subscriptions, TF listener, and an inactive ROS timer. It does not
prepare the robot, enable streaming, or run a Task until application code
explicitly calls `PlannerNode.run_task()` or another inherited command method.
There is no Qt dependency and no runtime mock implementation in this package.

## Architecture

```text
camera / detector
  -> /rby1/perception/object_pose (rby1_msgs/DetectedObjectPose)
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
immutable command types and `Task` builder from `rby1_control`; the wire schema
is not duplicated.

The two `task.py` modules have deliberately different jobs. The control
package keeps the copied, user-editable static Scenario catalog for the debug
UI. `rby1_planner.task` instead contains a runtime factory that composes a
validated camera observation with a caller-supplied object-to-end-effector
offset. Quaternion-to-RPY conversion uses the same convention and degree units
as the control backend.

The inherited command API includes `set_velocity`, `stop`, `prepare_robot`,
power/servo/stream/control-manager requests, joint and Cartesian moves,
motion cancellation, `start_task_command`, and Task status polling.
`PlannerTaskRunner` executes `TaskDefinition` instances sequentially with a
50 ms ROS timer. It retains the UI runner's fresh-state, EMO, collision,
control-state, command-result, and distance-aware timeout checks without using
a Qt event loop.

## Camera contract

`CameraClient` subscribes with best-effort, volatile, depth-5 sensor semantics.
Each input must have:

- a nonempty `object_id` and `header.frame_id`;
- a valid ROS timestamp;
- finite metre position values;
- a finite, normalized x/y/z/w quaternion;
- confidence in the inclusive range `[0.0, 1.0]`.

It caches the newest valid observation per object ID, ignores out-of-order
samples, rejects stale/future-dated or low-confidence data at query time, and
uses `tf2_ros.Buffer.lookup_transform()` at the original detection timestamp.
The returned `ObjectObservation` is deeply immutable: position and quaternion
are tuples rather than mutable ROS message fields.

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

## Building and running a camera-derived Task

```python
from rby1_planner.task import build_observation_cartesian_task

observation = node.camera.require_observation('tag_0')
task = build_observation_cartesian_task(
    name='move_to_planned_grasp',
    observation=observation,
    arm='right_arm',
    object_to_end_effector_position=object_to_ee_xyz,
    object_to_end_effector_orientation_xyzw=object_to_ee_quaternion,
    tcp_motion=validated_tcp_motion,
)
node.run_task(task)
```

The offset, arm, and motion limits have no defaults: planner application code
must supply calibrated values after checking the intended observation,
workspace, approach path, and collision policy. The factory adds one absolute
Cartesian command and returns the canonical immutable `TaskDefinition`.
Merely launching `rby1_planner` never runs a Task.

## Run

Build `rby1_control`, `rby1_msgs`, and this package in the same workspace, then
start the production backend and the idle planner in separate terminals:

```bash
ros2 launch rby1_control control.launch.py
ros2 launch rby1_planner planner.launch.py
```

The planner launch does not start a camera driver, perception producer,
control backend, or debug UI. Those lifecycle choices stay explicit.

The control debug UI and planner publish to the same command topic. Until the
backend has a controller-ownership/arbitration policy, do not issue motion from
both frontends at the same time. Also do not launch the old
`rby1_control_ui` backend alongside `rby1_control`; two subscribers on the same
command topic would execute each command twice.

## Current integration prerequisite

The existing AprilTag launch consumes RealSense images but does not currently
adapt `apriltag_ros` detections into `rby1_msgs/DetectedObjectPose`. The custom
message is also present only in the separate, uncommitted `rby1-ros2` worktree
at the time this package was created. Real-camera planning therefore requires
that interface generation and a producer/adapter to be completed outside this
package. `rby1_planner` intentionally does not modify `rby1-ros2` or
`rby1_camera`.

The hardware bringup must also publish a calibrated TF chain from the
RealSense optical frame to `base`. That fixed camera-mount transform is not
defined in this repository; without it, CameraClient safely returns no
base-frame observation.

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
