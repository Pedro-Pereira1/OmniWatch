from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
import os

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_share = get_package_share_directory('simple_car')
    shell_script = os.path.join(pkg_share, 'launch', 'spawn_entity.sh')
    world = os.path.join(pkg_share, 'launch', 'aqui.sdf')

    launch_description = LaunchDescription()

    # Launch Gazebo with the world
    launch_description.add_action(
        ExecuteProcess(
            cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_factory.so', world],
            output='screen'
        )
    )

    # Spawn cars and run waypoint_follower for each
    car_positions = {
        'car_1': ('1', '1'),
        'car_2': ('18', '18'),
        'car_3': ('1', '18'),
        'car_4': ('18', '1'),
    }

    for car_name, (x, y) in car_positions.items():
        # Spawn the car
        launch_description.add_action(
            ExecuteProcess(
                cmd=[shell_script, car_name, x, y],
                output='screen'
            )
        )

        # Launch the waypoint_follower node for the car
        launch_description.add_action(
            Node(
                package='simple_car',
                executable='waypoint_follower',
                name=f'{car_name}_waypoint_follower',
                output='screen',
                parameters=[{'car_name': car_name}]
            )
        )

    return launch_description
