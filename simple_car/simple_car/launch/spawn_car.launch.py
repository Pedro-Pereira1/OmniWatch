from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
import os

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_share = get_package_share_directory('simple_car')
    shell_script = os.path.join(pkg_share, 'launch', 'spawn_entity.sh')

    launch_description = LaunchDescription([
        ExecuteProcess(
            cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_factory.so'],
            output='screen'
        ),
        ExecuteProcess(
                cmd=[shell_script, 'car_1', '1', '1'],
                output='screen'
            )
    ])

    # Adiciona os carros de car_1 a car_20
    #for i in range(1, 7):
        #car_name = f'car_{i}'
        #launch_description.add_action(
            
        #)

    return launch_description
