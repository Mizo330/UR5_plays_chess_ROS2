from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch_ros.actions import Node
from launch.event_handlers import OnProcessExit

THIS_PACKAGE = "chess_gazebo_world"

def generate_launch_description():
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('ur_chess'), 'config', 'ur_chess_config.yaml'
        ]),
        description='Path to config YAML file'
    )
    config_file = LaunchConfiguration('config_file')
    
    world_gen_node = Node(
        package=THIS_PACKAGE,
        executable='generate_chess_sdf',
        name='generate_chess_sdf',
        output='screen',
        parameters=[config_file]
    )
    
    # Include UR5 robot with MoveIt and Gazebo world
    ur5_with_world = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare("ur_simulation_gz"),
                "launch",
                "ur_sim_moveit.launch.py"
            ])
        ]),
        launch_arguments={
            "ur_type": "ur5e",
            "launch_rviz": "true",
            "world_file": PathJoinSubstitution([
                FindPackageShare(THIS_PACKAGE),
                "worlds",
                "chess_room.world"
            ]),
            "simulation_controllers": PathJoinSubstitution([
                FindPackageShare("ur_simulation_gz"),
                "config",
                "ur_controllers.yaml"
            ]),
        }.items()
    )

    #wait for the world to generate before sim launch
    launch_sim_after_world_gen = RegisterEventHandler(
        OnProcessExit(
            target_action=world_gen_node,
            on_exit=[ur5_with_world]
        )
    )

    return LaunchDescription([
        config_file_arg,
        world_gen_node,
        launch_sim_after_world_gen,
    ])
