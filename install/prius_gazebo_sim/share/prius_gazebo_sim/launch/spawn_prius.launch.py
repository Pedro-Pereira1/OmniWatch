import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess


def generate_launch_description():
    sdf_path = os.path.expanduser('~/.gazebo/models/prius_hybrid/model.sdf')

    return LaunchDescription([
        ExecuteProcess(
            cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_factory.so'],
            output='screen'
        ),
        ExecuteProcess(
            cmd=[
                'ros2', 'run', 'gazebo_ros', 'spawn_entity.py',
                '-file', sdf_path,
                '-entity', 'prius_hybrid'
            ],
            output='screen'
        ),
    ])