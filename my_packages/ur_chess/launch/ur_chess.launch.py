from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, RegisterEventHandler, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnProcessExit

THIS_PACKAGE = "ur_chess"

def generate_launch_description():
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=PathJoinSubstitution([
            FindPackageShare(THIS_PACKAGE), 'config', 'ur_chess_config.yaml'
        ]),
        description='Path to config YAML file'
    )
    config_file = LaunchConfiguration('config_file')

    moveit_default_params = {
        'moveit_cpp_cfg': PathJoinSubstitution([
            FindPackageShare(THIS_PACKAGE), 'config', 'moveit_cpp.yaml'
        ]),
        'moveit_cpp_srdf': PathJoinSubstitution([
            FindPackageShare(THIS_PACKAGE), 'config', 'moveit_cpp.srdf'
        ]),
        'moveit_controller_cfg': PathJoinSubstitution([
            FindPackageShare('ur_moveit_config'), 'config', 'moveit_controllers.yaml'
        ])
    }


    # Declare launch argument
    mode_arg = DeclareLaunchArgument(
        'mode',
        default_value='FishVFish',
        description='Select what mode you want to play in.',
        choices=['FishVFish', 'PVP', 'P(b)VFish', 'P(w)VFish']
    )

    # Get launch configuration substitution
    mode = LaunchConfiguration('mode')

    #add sim launch:
    ur5_sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('chess_gazebo_world'), 'launch', 'chess_world_with_ur.launch.py'
            ])
        ),
        launch_arguments={
            'config_file': config_file
        }.items()  
    )
    
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
            parameters=[
                moveit_default_params,
                config_file
            ],
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
            parameters=[{'mode': mode}, config_file],
        )
    ]

    return LaunchDescription([mode_arg, 
                              config_file_arg,
                              ur5_sim_launch
                            ]+nodes)
