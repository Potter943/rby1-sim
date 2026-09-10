"""User-editable RB-Y1 Task definitions.

Edit this file in the IDE, then press ``Reload Tasks`` in the Scenario tab.
Get Q and Get TCP return values in exactly the units expected here.
"""
from __future__ import annotations

from .task_commands import Task, list_sum

# --------------------------------------------------------------------------------------------
# 기본 pose들
# --------------------------------------------------------------------------------------------

def stand_pose() -> Task:
    """Move torso and both arms together to a standing up posture."""

    task = Task("stand_pose",)
    joint_motion = [5.0, 0.20, 0.20]
    task.whole_body_joint_absolute(
        torso=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        right_arm=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        left_arm=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        joint_motion=joint_motion,
    )
    return task


def ready_pose() -> Task:
    """Move torso and both arms together to the Cartesian-ready posture."""

    task = Task(
        "ready_pose",
        "Moves torso and both arms together to the SDK Cartesian-ready pose.",
    )
    joint_motion = [5.0, 0.20, 0.20]
    task.whole_body_joint_absolute(
        torso=[0.0, 45.0, -90.0, 45.0, 0.0, 0.0],
        right_arm=[0.0, -5.0, 0.0, -120.0, 0.0, 40.0, 0.0],
        left_arm=[0.0, 5.0, 0.0, -120.0, 0.0, 40.0, 0.0],
        joint_motion=joint_motion,
    )
    return task

def object_gripping_initial_pose() -> Task:
    """Move to the measured initial posture for object gripping Tasks."""

    task = Task(
        "object_gripping_initial_pose",
        "Measured initial posture with both grippers facing downward.",
    )
    joint_motion = [2.0, 2.00, 2.00]
    task.whole_body_joint_absolute(
        torso=[0.059126, 45.376237, -90.963053, 45.154560, 0.021390, -0.000280,],
        right_arm=[-34.913810, -63.837794, 46.172754, -106.640370, 16.002640, 45.214678, 83.061208,],
        left_arm=[-33.643824, 63.218668, -44.626985, -106.563932, -17.501234, 44.556740, -84.095907],
        joint_motion=joint_motion
    )
    return task

# --------------------------------------------------------------------------------------------
# 움직임 관련 task들
# --------------------------------------------------------------------------------------------

def torso_rotation() -> Task:
    task = Task("torso_rotation")

    task.joint_absolute(
        "torso", [0.065071, 45.375357, -90.960025, 45.153579, 0.027261, -90.0],  # 6개, deg
        [4.0, 1.0, 1.0],
    )

    return task
def base_forward_example() -> Task:
    task = Task("base_forward_example")
    task.base_velocity(vx = 0.20, vy = 0.0, wz = 0.0, seconds=3.0)
    return task

def base_sideways_example() -> Task:
    task = Task("base_sideways_example")
    task.base_velocity(vx = 0.0, vy = -0.15, wz = 0.0, seconds=4.0)
    return task

def right_tcp_example() -> Task:

    task = Task("right_tcp_example",)
    pos = [0.437629, -0.234868, 1.011805, -26.707221, -78.839014, 26.270230]
    move_z = 0.10
    # tcp_motion = [minimum_time(s), linear_velocity(m/s), angular_velocity(rad/s), acceleration_scaling(0.0~1.0)]
    tcp_motion = [5.0, 0.05, 0.20, 0.30]

    task.linear_absolute("right_arm", list_sum(pos, [0.0, 0.0, move_z, 0.0, 0.0, 0.0]), tcp_motion = [5.0, 0.05, 0.20, 0.30])
    task.delay(750)
    task.linear_absolute("right_arm", pos, tcp_motion)
    return task


def right_joint_example() -> Task:
    """Move the right arm to a saved bent-elbow example posture."""

    task = Task("right_joint_example",)
    q = [-10.001416, -20.000159, -0.000520, -40.001279, -0.000064, -0.002350, -0.001015]
    # joint_motion = [minimum_time(s), velocity_limit(rad/s), acceleration_limit(rad/s^2)]
    joint_motion = [10.0, 0.20, 0.20]
    task.joint_absolute("right_arm", q, joint_motion)
    return task

def object_handover_demo() -> Task:
    """Pick with the right hand and hand the object to the left hand."""
    task = Task("object_handover_demo",)
    task.extend(ready_pose())
    task.extend(object_gripping_initial_pose())
    # 값 설정
    ready_left_arm = [-33.643824, 63.218668, -44.626985, -106.563932, -17.501234, 44.556740, -84.095907]
    # joint_motion = [minimum_time(s), velocity_limit(rad/s), acceleration_limit(rad/s^2)]
    # tcp_motion = [minimum_time(s), linear_velocity(m/s), angular_velocity(rad/s), acceleration_scaling(0.0~1.0)]
    ready_joint_motion = [3.0, 1.00, 1.00]
    tcp_motion = [1.0, 4.00, 4.00, 1.00]
    pickup_distance = 0.10

    # FK results for the measured initial Q, in base-frame
    # [x(m), y(m), z(m), roll(deg), pitch(deg), yaw(deg)].
    ready_right_tcp = [0.374015, -0.243156, 0.920902, 0.004552, -0.023684, 89.899989]
    ready_left_tcp = [0.374034, 0.243053, 0.921057, 0.008670, 0.007028, -89.997544]
    # Base-frame X rotations make the downward-facing grippers face inward.
    right_facing_left_rpy = [96.757919, -89.899299, -6.755644]
    left_facing_right_rpy = [-19.257234, -89.992555, -70.734096]

    # The previous +/-0.04 m targets allowed the hands to overlap. Doubling
    # the TCP separation leaves 0.16 m between the two handover targets.
    handover_half_gap = 0.08
    right_handover_tcp = [
        ready_right_tcp[0], -handover_half_gap, ready_right_tcp[2],
        *right_facing_left_rpy,
    ]
    left_handover_tcp = [
        ready_left_tcp[0], handover_half_gap, ready_left_tcp[2],
        *left_facing_right_rpy,
    ]

    # Right hand: descend, assume grasp, lift, face left, and move to center.
    task.linear_absolute(
        "right_arm", list_sum(ready_right_tcp, [0.0, 0.0, -pickup_distance, 0.0, 0.0, 0.0]), tcp_motion,
    )
    task.delay(750)  # Placeholder: close the right gripper.
    task.linear_absolute("right_arm", ready_right_tcp, tcp_motion)
    task.linear_absolute(
        "right_arm",
        [*ready_right_tcp[:3], *right_facing_left_rpy],
        tcp_motion,
    )
    task.linear_absolute("right_arm", right_handover_tcp, tcp_motion)

    # Left hand: face right and approach the same handover area from the left.
    task.linear_absolute(
        "left_arm", [*ready_left_tcp[:3], *left_facing_right_rpy], tcp_motion,
    )
    task.linear_absolute("left_arm", left_handover_tcp, tcp_motion)
    task.delay(750)  # Placeholder: close the left gripper.
    task.delay(750)  # Placeholder: open the right gripper.
    task.extend(base_sideways_example())
    # Return the left arm to its ready pose, lower it, and assume release.
    task.joint_absolute("left_arm", ready_left_arm, ready_joint_motion)

    task.linear_absolute(
        "left_arm",
        list_sum(ready_left_tcp, [0.0, 0.0, -pickup_distance, 0.0, 0.0, 0.0]),
        tcp_motion,
    )
    task.delay(750)  # Placeholder: open the left gripper.
    return task

def object_handover_demo2() -> Task:
    """Pick with the right hand, rotate torso and place the object down."""
    task = Task("object_handover_demo2",)
    task.extend(object_gripping_initial_pose())
    # 값 설정
    tcp_motion = [1.0, 4.00, 4.00, 1.00]
    pickup_distance = 0.10

    ready_right_tcp = [0.374015, -0.243156, 0.920902, 0.004552, -0.023684, 89.899989]
    # Right hand: descend, assume grasp, lift, face left, and move to center.
    task.linear_absolute(
        "right_arm", list_sum(ready_right_tcp, [0.0, 0.0, -pickup_distance, 0.0, 0.0, 0.0]), tcp_motion,
    )
    task.delay(750)  # Placeholder: close the right gripper.
    task.linear_absolute("right_arm", ready_right_tcp, tcp_motion)
    task.extend(torso_rotation())
    task.linear_relative(
            "right_arm", [0.0, 0.0, -pickup_distance, 0.0, 0.0, 0.0], tcp_motion,
        )
    task.delay(750)  # Placeholder: close the right gripper.
    task.linear_relative("right_arm", [0.0, 0.0, pickup_distance, 0.0, 0.0, 0.0], tcp_motion)

    return task

def combined_example() -> Task:
    """Example of joining Tasks in the same style as KR's extend."""

    task = Task("combined_example")
    task.extend(right_tcp_example())
    task.extend(right_joint_example())
    task.extend(ready_pose())
    return task
# --------------------------------------------------------------------------------------------

def build_tasks() -> dict[str, Task]:
    """Only Tasks registered here appear in the Scenario Task List."""

    return {
        "stand_pose": stand_pose(),
        "ready_pose": ready_pose(),
        "right_tcp_example": right_tcp_example(),
        "right_joint_example": right_joint_example(),
        "combined_example": combined_example(),
        "object_gripping_initial_pose": object_gripping_initial_pose(),
        "object_handover_demo": object_handover_demo(),
        "base_forward_example": base_forward_example(),
        "base_sideways_example": base_sideways_example(),
        "torso_rotation": torso_rotation(),
        "object_handover_demo2": object_handover_demo2(),
    }


__all__ = ["Task", "list_sum", "build_tasks"]
