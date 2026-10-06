import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():

    robot_description_mappings = {
        "safety_limits": "true",
        "safety_pos_margin": "0.15",
        "safety_k_position": "20",
        "name": "ur",
        "ur_type": "ur10e",
        "tf_prefix": "",
        "simulation_controllers": os.path.join(
            get_package_share_directory("sorting_arm"),
            "config",
            "ur_controllers.yaml",
        ),
    }

    moveit_config = (
        MoveItConfigsBuilder(
            "sorting_arm",
            package_name="sorting_arm_moveit_config",
        )
        .robot_description(
            file_path=os.path.join(
                get_package_share_directory("sorting_arm_gazebo"),
                "urdf",
                "sorting_cell.urdf.xacro",
            ),
            mappings=robot_description_mappings,
        )
        .trajectory_execution(
            file_path="config/moveit_controllers.yaml"
        )
        .to_moveit_configs()
    )

    planner_node = Node(
        package="sorting_arm_motion_planner",
        executable="motion_planner",
        output="screen",
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
            {"use_sim_time": True},
        ],
    )

    return LaunchDescription([planner_node])
