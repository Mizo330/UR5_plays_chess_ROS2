from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs):
    # Initialize Arguments
    robot_ip = LaunchConfiguration("robot_ip")
    safety_limits = LaunchConfiguration("safety_limits")
    # General arguments
    # controllers_file = LaunchConfiguration("controllers_file")
    # description_file = LaunchConfiguration("description_file")
    moveit_launch_file = LaunchConfiguration("moveit_launch_file")

    ur_control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("ur_robot_driver"), "launch", "ur_control.launch.py"]
            )
        ),
        launch_arguments={
            "ur_type": "ur5e",
            "robot_ip": robot_ip,
            "safety_limits": safety_limits,
            # "controllers_file": controllers_file,
            # "description_file": description_file,
            "launch_rviz": "false",
        }.items(),
    )

    ur_moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("ur_moveit_config"),"launch","ur_moveit.launch.py",
                ]
            ),),
        launch_arguments={
            "ur_type": "ur5e",
            "launch_rviz": "true",
        }.items(),
    )

    nodes_to_launch = [
        ur_control_launch,
        ur_moveit_launch,
    ]

    return nodes_to_launch


def generate_launch_description():
    declared_arguments = []
    # UR specific arguments
    declared_arguments.append(
        DeclareLaunchArgument(
            "robot_ip",
            description="Ip for the robot",
            default_value="192.168.50.221",
        )
    )
    
    declared_arguments.append(
        DeclareLaunchArgument(
            "safety_limits",
            default_value="true",
            description="Enables the safety limits controller if true.",
        )
    )
    # # General arguments
    # declared_arguments.append(
    #     DeclareLaunchArgument(
    #         "controllers_file",
    #         default_value=PathJoinSubstitution(
    #             [FindPackageShare("ur_simulation_gz"), "config", "ur_controllers.yaml"]
    #         ),
    #         description="Absolute path to YAML file with the controllers configuration.",
    #     )
    # )
    # declared_arguments.append(
    #     DeclareLaunchArgument(
    #         "description_file",
    #         default_value=PathJoinSubstitution(
    #             [FindPackageShare("ur_simulation_gz"), "urdf", "ur_gz.urdf.xacro"]
    #         ),
    #         description="URDF/XACRO description file (absolute path) with the robot.",
    #     )
    # )
    declared_arguments.append(
        DeclareLaunchArgument(
            "moveit_launch_file",
            default_value=PathJoinSubstitution(
                [
                    FindPackageShare("ur_moveit_config"),
                    "launch",
                    "ur_moveit.launch.py",
                ]
            ),
            description="Absolute path for MoveIt launch file, part of a config package with robot SRDF/XACRO files. Usually the argument "
            "is not set, it enables use of a custom moveit config.",
        )
    )

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
