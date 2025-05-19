from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


THIS_PACKAGE = "ur_chess"


def generate_launch_description():
    declared_arguments = []
    # UR specific arguments
    declared_arguments.append(
        Node(
            package=THIS_PACKAGE,
            executable='game_manager',
            name='game_manager',
            output='screen',
        )
    )
    
    declared_arguments.append(
        Node(
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
        )
    )
    
    declared_arguments.append(
        Node(
            package=THIS_PACKAGE,
            executable='gui',
            name='gui',
            output='screen',
        )
    )
    
    declared_arguments.append(
        Node(
            package=THIS_PACKAGE,
            executable='stockfish_node',
            name='stockfish_node',
            output='screen',
        )
    )

    return LaunchDescription(declared_arguments)