import launch
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='pick_and_place_two_robots',
            executable='server.py',
            name='server',
            parameters=[
                '/home/victor/ur_rtde/TFG/src/pick_and_place_two_robots/launch/config/args_for_launch.yaml' 
            ]
        ),
        Node(
            package='pick_and_place_two_robots',
            executable='client1.py',
            name='client'
        ),
        Node(
            package='pick_and_place_two_robots',
            executable='client2.py',
            name='client2'
        ),
    ])

