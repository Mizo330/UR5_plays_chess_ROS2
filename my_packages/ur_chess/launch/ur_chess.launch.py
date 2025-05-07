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
            parameters=[{
                'config_file': PathJoinSubstitution(
                    [FindPackageShare(THIS_PACKAGE), 'config', 'board_layout.yaml']
                )
            }],
        )
    )
    
    declared_arguments.append(
        Node(
            package=THIS_PACKAGE,
            executable='move_controller',
            name='move_controller',
            output='screen',
            parameters=[{
                'constraints_file': PathJoinSubstitution(
                    [FindPackageShare(THIS_PACKAGE), 'config', 'planning_constaints.yaml']
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

    return LaunchDescription(declared_arguments)