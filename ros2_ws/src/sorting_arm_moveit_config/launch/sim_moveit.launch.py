"""MoveIt (move_group + RViz) against the running integrated Gazebo simulation.

Start the simulation first:
  ros2 launch sorting_arm_gazebo sorting_sim.launch.py
then:
  ros2 launch sorting_arm_moveit_config sim_moveit.launch.py

robot_state_publisher, ros2_control and the controllers are owned by the Gazebo launch;
this file only adds move_group and RViz, both on simulation time.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    # Same xacro and arguments that ur_simulation_gz/ur_sim_control.launch.py uses for the Gazebo
    # robot (via sorting_arm_gazebo/sorting_sim.launch.py), so robot_description is identical,
    # including the pedestal mount world->base_link.
    robot_description_mappings = {
        "safety_limits": "true",
        "safety_pos_margin": "0.15",
        "safety_k_position": "20",
        "name": "ur",
        "ur_type": "ur10e",
        "tf_prefix": "",
        "simulation_controllers": os.path.join(
            get_package_share_directory("sorting_arm"), "config", "ur_controllers.yaml"
        ),
    }

    moveit_config = (
        MoveItConfigsBuilder("sorting_arm", package_name="sorting_arm_moveit_config")
        .robot_description(
            file_path=os.path.join(
                get_package_share_directory("sorting_arm_gazebo"),
                "urdf",
                "sorting_cell.urdf.xacro",
            ),
            mappings=robot_description_mappings,
        )
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .to_moveit_configs()
    )

    use_sim_time = {"use_sim_time": True}

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config.to_dict(), use_sim_time],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2_moveit",
        output="log",
        arguments=[
            "-d",
            os.path.join(
                get_package_share_directory("sorting_arm_moveit_config"), "config", "moveit.rviz"
            ),
        ],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
            use_sim_time,
        ],
        condition=IfCondition(LaunchConfiguration("launch_rviz")),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("launch_rviz", default_value="true"),
            move_group_node,
            rviz_node,
        ]
    )
