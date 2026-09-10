from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = Path(get_package_share_directory('rby1_planner'))
    default_config = str(package_share / 'config' / 'default.yaml')

    namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='rby1',
        description='Namespace shared with the rby1_control backend.',
    )
    config_arg = DeclareLaunchArgument(
        'config',
        default_value=default_config,
        description='Path to the planner ROS parameter YAML file.',
    )
    planner_node = Node(
        package='rby1_planner',
        executable='planner',
        name='rby1_planner',
        namespace=LaunchConfiguration('namespace'),
        parameters=[LaunchConfiguration('config')],
        output='screen',
        emulate_tty=True,
    )

    return LaunchDescription([
        namespace_arg,
        config_arg,
        planner_node,
    ])
