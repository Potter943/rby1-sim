"""Launch the RB-Y1 backend with its Qt command-validation frontend."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = Path(get_package_share_directory('rby1_control'))
    default_config = str(package_share / 'config' / 'default.yaml')

    namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='rby1',
        description='ROS namespace for the debug frontend and backend.',
    )
    config_arg = DeclareLaunchArgument(
        'config',
        default_value=default_config,
        description='Path to the control ROS parameter YAML file.',
    )

    backend_node = Node(
        package='rby1_control',
        executable='control_backend',
        name='rby1_control_backend',
        namespace=LaunchConfiguration('namespace'),
        parameters=[LaunchConfiguration('config')],
        output='screen',
        emulate_tty=True,
    )
    ui_node = Node(
        package='rby1_control',
        executable='control_ui',
        name='rby1_control_debug_ui',
        namespace=LaunchConfiguration('namespace'),
        parameters=[LaunchConfiguration('config')],
        output='screen',
        emulate_tty=True,
    )

    return LaunchDescription([
        namespace_arg,
        config_arg,
        backend_node,
        ui_node,
    ])
