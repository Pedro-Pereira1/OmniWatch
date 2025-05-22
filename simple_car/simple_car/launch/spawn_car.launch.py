from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
import os

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_share = get_package_share_directory('simple_car')
    sdf_file = os.path.join(pkg_share, 'models', 'simple_car', 'model.sdf')

    return LaunchDescription([
        ExecuteProcess(
            cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_factory.so'],
            output='screen'
        ),
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=[
                '-entity', 'simple_car',
                '-file', sdf_file,
                '-x', '0', '-y', '0', '-z', '0.1'
            ],
            output='screen'
        ),
    ])
