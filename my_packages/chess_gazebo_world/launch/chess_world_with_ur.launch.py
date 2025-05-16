from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
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

    return LaunchDescription([ur5_with_world])
