import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node


def generate_launch_description():

    # Find installed package directories
    pkg_sorting_arm_gazebo = get_package_share_directory(
        'sorting_arm_gazebo'
    )

    pkg_ros_gz_sim = get_package_share_directory(
        'ros_gz_sim'
    )

    # Paths to our resources
    world_file = os.path.join(
        pkg_sorting_arm_gazebo,
        'worlds',
        'sorting_world.sdf'
    )

    bridge_config = os.path.join(
        pkg_sorting_arm_gazebo,
        'config',
        'bridge.yaml'
    )

    models_path = os.path.join(
        pkg_sorting_arm_gazebo,
        'models'
    )

    # Allow Gazebo to find our custom models
    set_gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=models_path
    )

    # Start Gazebo with our world
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                pkg_ros_gz_sim,
                'launch',
                'gz_sim.launch.py'
            )
        ),
        launch_arguments={
            'gz_args': '-r ' + world_file
        }.items()
    )

    # Bridge camera images from Gazebo to ROS 2
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ros_gz_bridge',
        parameters=[
            {'config_file': bridge_config}
        ],
        output='screen'
    )

    return LaunchDescription([
        set_gz_resource_path,
        gazebo,
        bridge
    ])