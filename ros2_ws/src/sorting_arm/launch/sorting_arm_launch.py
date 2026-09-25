from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
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
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(ur_sim),
            launch_arguments={
                "ur_type": "ur10e",
                "description_file": PathJoinSubstitution([pkg, "urdf", "sorting_arm.urdf.xacro"]),
                "world_file": PathJoinSubstitution([pkg, "worlds", "environment.sdf"]),
                "controllers_file": PathJoinSubstitution([pkg, "config", "ur_controllers.yaml"]),
            }.items(),
        ),
        gripper_spawner,
        
    ])