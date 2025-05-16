from launch import LaunchDescription
from launch_ros.actions import Node

#Only if spawning is needed

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='chess_gazebo_world',
            executable='spawn_pieces',
            name='spawn_pieces_node',
            output='screen'
        )
    ])
