from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, Command
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
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
                FindPackageShare("chess_gazebo_world"),
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

    generate_chess_sdf = ExecuteProcess(
        cmd=[PathJoinSubstitution([
            FindPackageShare("chess_gazebo_world"),
            "chess_gazebo_world",
            "generate_chess_sdf.py"
        ])],
        shell=True,
        output="screen"
    )

    return LaunchDescription([
        ur5_with_world,
        generate_chess_sdf,
    ])
