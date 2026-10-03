import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    AppendEnvironmentVariable,
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    RegisterEventHandler,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.event_handlers import OnProcessExit

from launch_ros.actions import Node


def generate_launch_description():
    """Full sorting cell: Manu's UR10e + AG95 + ros2_control in Krish's world with the camera."""

    pkg_sorting_arm = get_package_share_directory('sorting_arm')
    pkg_sorting_arm_gazebo = get_package_share_directory('sorting_arm_gazebo')

    # Allow Gazebo to find our custom models (append so other paths are kept)
    set_gz_resource_path = AppendEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=os.path.join(pkg_sorting_arm_gazebo, 'models')
    )

    randomize_world = ExecuteProcess(
        cmd=[
            'python3',
            os.path.join(
                pkg_sorting_arm_gazebo,
                'scripts',
                'randomize_objects.py'
            ),
            os.path.join(
                pkg_sorting_arm_gazebo,
                'worlds',
                'sorting_world.sdf'
            ),
            '/tmp/sorting_world_randomized.sdf',
        ],
        output='screen',
    )

    # Robot, controllers and Gazebo, via the existing sorting_arm launch
    robot_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_sorting_arm, 'launch', 'sorting_arm_launch.py')
        ),
        launch_arguments={
            'description_file': os.path.join(
                pkg_sorting_arm_gazebo, 'urdf', 'sorting_cell.urdf.xacro'
            ),
            'world_file': '/tmp/sorting_world_randomized.sdf',
            'launch_rviz': LaunchConfiguration('launch_rviz'),
        }.items()
    )

    start_sim_after_randomization = RegisterEventHandler(
        OnProcessExit(
            target_action=randomize_world,
            on_exit=[robot_sim],
        )
    )

    # Bridge camera images from Gazebo to ROS 2
    camera_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='camera_bridge',
        parameters=[
            {'config_file': os.path.join(pkg_sorting_arm_gazebo, 'config', 'bridge.yaml')}
        ],
        output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'launch_rviz', default_value='true',
            description="Start ur_simulation_gz's RViz (set false when using MoveIt's RViz)."
        ),
        set_gz_resource_path,
        randomize_world,
        start_sim_after_randomization,
        camera_bridge
    ])
