from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

gripper_spawner = Node(
    package="controller_manager",
    executable="spawner",
    arguments=["gripper_controller",
               "--controller-manager", "/controller_manager",
               "--controller-manager-timeout", "120"],
)

def generate_launch_description():
    pkg = FindPackageShare("sorting_arm")
    ur_sim = PathJoinSubstitution(
        [FindPackageShare("ur_simulation_gz"), "launch", "ur_sim_control.launch.py"]
    )
    return LaunchDescription([
        DeclareLaunchArgument(
            "description_file",
            default_value=PathJoinSubstitution([pkg, "urdf", "sorting_arm.urdf.xacro"]),
        ),
        DeclareLaunchArgument(
            "world_file",
            default_value=PathJoinSubstitution([pkg, "worlds", "environment.sdf"]),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(ur_sim),
            launch_arguments={
                "ur_type": "ur10e",
                "description_file": LaunchConfiguration("description_file"),
                "world_file": LaunchConfiguration("world_file"),
                "controllers_file": PathJoinSubstitution([pkg, "config", "ur_controllers.yaml"]),
            }.items(),
        ),
        gripper_spawner,
        
    ])