from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

THIS_PACKAGE = "ur_chess"

def generate_launch_description():
    # Declare launch argument
    mode_arg = DeclareLaunchArgument(
        'mode',
        default_value='FishVFish',
        description='Select what mode you want to play in.',
        choices=['FishVFish', 'PVP', 'P(b)VFish', 'P(w)VFish']
    )

    # Get launch configuration substitution
    mode = LaunchConfiguration('mode')

    # Define nodes
    nodes = [
        Node(
            namespace=THIS_PACKAGE,
            package=THIS_PACKAGE,
            executable='game_manager',
            name='game_manager',
            output='screen',
        ),
        Node(
            namespace=THIS_PACKAGE,
            package=THIS_PACKAGE,
            executable='moveit_controller',
            name='moveit_controller',
            output='screen',
            parameters=[{
                'moveit_cpp_cfg': PathJoinSubstitution(
                    [FindPackageShare(THIS_PACKAGE), 'config', 'moveit_cpp.yaml']
                ),
                'moveit_cpp_srdf': PathJoinSubstitution(
                    [FindPackageShare(THIS_PACKAGE), 'config', 'moveit_cpp.srdf']
                ),
                'moveit_controller_cfg': PathJoinSubstitution(
                    [FindPackageShare("ur_moveit_config"), 'config', 'moveit_controllers.yaml']
                ),
                'board_layout': PathJoinSubstitution(
                    [FindPackageShare(THIS_PACKAGE), 'config', 'board_layout.yaml']
                )
            }],
        ),
        Node(
            namespace=THIS_PACKAGE,
            package=THIS_PACKAGE,
            executable='gui',
            name='gui',
            output='screen',
            parameters=[{'mode': mode}],
        ),
        Node(
            namespace=THIS_PACKAGE,
            package=THIS_PACKAGE,
            executable='stockfish_node',
            name='stockfish_node',
            output='screen',
            parameters=[{'mode': mode}],
        )
    ]

    # Return launch description with the declared argument and all nodes
    return LaunchDescription([mode_arg] + nodes)
