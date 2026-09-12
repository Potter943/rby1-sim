"""Launch the mock object-pose producer for perception integration tests."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = Path(get_package_share_directory('rby1_camera'))
    default_config = str(package_share / 'config' / 'mock_object_pose.yaml')
    default_mount_config = str(
        package_share / 'config' / 'mock_camera_mount_tf.yaml'
    )

    namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='rby1',
        description='Namespace for the mock perception producer.',
    )
    config_arg = DeclareLaunchArgument(
        'config',
        default_value=default_config,
        description='Mock object-pose parameter file.',
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

    mock_publisher = Node(
        package='rby1_camera',
        executable='mock_object_pose_publisher',
        name='mock_object_pose_publisher',
        namespace=LaunchConfiguration('namespace'),
        parameters=[LaunchConfiguration('config')],
        output='screen',
        emulate_tty=True,
    )

    return LaunchDescription([
        namespace_arg,
        config_arg,
        mount_config_arg,
        camera_mount_tf,
        mock_publisher,
    ])
