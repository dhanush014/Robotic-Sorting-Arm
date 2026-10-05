from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

 
 
def generate_launch_description():
    pkg = FindPackageShare("sorting_arm")
    ur_sim = PathJoinSubstitution(
        [FindPackageShare("ur_simulation_gz"), "launch", "ur_sim_control.launch.py"]
    )
 
    ur_sim_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(ur_sim),
        launch_arguments={
            "ur_type": "ur10e",
            "description_file": PathJoinSubstitution([pkg, "urdf", "sorting_arm.urdf.xacro"]),
            "world_file": PathJoinSubstitution([pkg, "worlds", "environment.sdf"]),
            "controllers_file": PathJoinSubstitution([pkg, "config", "ur_controllers.yaml"]),
        }.items(),
    )
 
    gripper_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "gripper_controller",
            "--controller-manager", "/controller_manager",
            "--controller-manager-timeout", "120",
        ],
    )
 

    pick_place_node = Node(
        package="sorting_arm_tasks",
        executable="state_machine",
        output="screen",
        parameters=[{"use_sim_time": True}],
    )

    delayed_pick_place = RegisterEventHandler(
        OnProcessExit(
            target_action=gripper_spawner,
            on_exit=[pick_place_node],
        )
    )
 
    return LaunchDescription([
        ur_sim_include,
        gripper_spawner,
        delayed_pick_place,
        pick_place_node,
    ])