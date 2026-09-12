from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = Path(get_package_share_directory('rby1_camera'))
    config_file = str(package_share / 'config' / 'apriltag.yaml')
    default_mount_config = str(
        package_share / 'config' / 'camera_mount_tf.yaml'
    )

    namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='rby1',
        description='Namespace shared with the planner.',
    )
    mount_config_arg = DeclareLaunchArgument(
        'mount_config',
        default_value=default_mount_config,
        description='End-effector-to-camera static TF parameter file.',
    )

    camera_mount_tf = Node(
        package='rby1_camera',
        executable='camera_mount_tf_publisher',
        name='camera_mount_tf_publisher',
        namespace=LaunchConfiguration('namespace'),
        parameters=[LaunchConfiguration('mount_config')],
        output='screen',
    )

    apriltag_node = Node(
        package='apriltag_ros',
        executable='apriltag_node',
        name='apriltag_node',
        namespace=LaunchConfiguration('namespace'),
        parameters=[config_file],
        remappings=[
            ('image_rect', '/camera/camera/color/image_raw'),
            ('camera_info', '/camera/camera/color/camera_info'),
            # Keep the planner input identical for the real and mock launches.
            ('detections', 'perception/object_pose'),
        ],
        output='screen',
    )

    return LaunchDescription([
        namespace_arg,
        mount_config_arg,
        camera_mount_tf,
        apriltag_node,
    ])
