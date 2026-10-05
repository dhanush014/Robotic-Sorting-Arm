"""Full sorting run: simulation, MoveIt, motion planner, detector and the state machine.

  ros2 launch sorting_arm_moveit_config sim_state_machine.launch.py
  ros2 launch sorting_arm_moveit_config sim_state_machine.launch.py gazebo_gui:=false   # headless
  (add headless_rendering:=true when there is no display at all)

Everything is started in order, each stage waiting for the previous one:
  1. randomize_objects.py writes /tmp/sorting_world_randomized.sdf
  2. Gazebo with that world (gz sim -s when gazebo_gui:=false), robot_state_publisher,
     /clock and camera bridges, and the robot spawned into Gazebo
  3. joint_state_broadcaster, joint_trajectory_controller and gripper_controller
  4. move_group (+ optional RViz) and the motion planner node (/move_to_pose)
  5. planning_scene_setup, once the motion planner is connected to move_group
  6. the object detector
  7. the state machine
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    AppendEnvironmentVariable,
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    LogInfo,
    RegisterEventHandler,
    Shutdown,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit, OnProcessIO, OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import IfElseSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder

RANDOMIZED_WORLD = "/tmp/sorting_world_randomized.sdf"  # also read by planning_scene_setup
PLANNER_READY_MSG = b"Motion planner service /move_to_pose started"


def on_success(stage, actions):
    """OnProcessExit callback: start `actions` only if the process exited cleanly."""
    def handler(event, _context):
        if event.returncode == 0:
            return actions
        return [
            LogInfo(msg=f"{stage} failed (exit code {event.returncode}), shutting down"),
            Shutdown(reason=f"{stage} failed"),
        ]
    return handler


def generate_launch_description():
    pkg_sorting_arm = get_package_share_directory("sorting_arm")
    pkg_gazebo = get_package_share_directory("sorting_arm_gazebo")
    pkg_moveit_config = get_package_share_directory("sorting_arm_moveit_config")

    gazebo_gui = LaunchConfiguration("gazebo_gui")
    launch_rviz = LaunchConfiguration("launch_rviz")
    show_camera = LaunchConfiguration("show_camera")
    use_sim_time = {"use_sim_time": True}

    # One robot description for Gazebo, robot_state_publisher, move_group and the planner.
    moveit_config = (
        MoveItConfigsBuilder("sorting_arm", package_name="sorting_arm_moveit_config")
        .robot_description(
            file_path=os.path.join(pkg_gazebo, "urdf", "sorting_cell.urdf.xacro"),
            mappings={
                "safety_limits": "true",
                "safety_pos_margin": "0.15",
                "safety_k_position": "20",
                "name": "ur",
                "ur_type": "ur10e",
                "tf_prefix": "",
                "simulation_controllers": os.path.join(
                    pkg_sorting_arm, "config", "ur_controllers.yaml"
                ),
            },
        )
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .to_moveit_configs()
    )

    # 1. Randomize the world
    randomize_world = ExecuteProcess(
        cmd=[
            "python3",
            os.path.join(pkg_gazebo, "scripts", "randomize_objects.py"),
            os.path.join(pkg_gazebo, "worlds", "sorting_world.sdf"),
            RANDOMIZED_WORLD,
        ],
        output="screen",
    )

    # 2. Gazebo with the randomized world, the robot and the bridges
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("ros_gz_sim"), "launch", "gz_sim.launch.py")
        ),
        launch_arguments={
            "gz_args": [
                IfElseSubstitution(gazebo_gui, if_value="", else_value="-s "),
                IfElseSubstitution(
                    LaunchConfiguration("headless_rendering"),
                    if_value="--headless-rendering ", else_value="",
                ),
                f"-r -v 4 {RANDOMIZED_WORLD}",
            ],
            "on_exit_shutdown": "true",
        }.items(),
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[moveit_config.robot_description, use_sim_time],
    )

    clock_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="clock_bridge",
        arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"],
        output="screen",
    )

    camera_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="camera_bridge",
        parameters=[{"config_file": os.path.join(pkg_gazebo, "config", "bridge.yaml")}],
        output="screen",
    )

    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=["-topic", "robot_description", "-name", "ur", "-allow_renaming", "true"],
    )

    # 3. Controllers (gz_ros2_control loads them from ur_controllers.yaml; this activates them)
    controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "joint_trajectory_controller",
            "gripper_controller",
            "--controller-manager", "/controller_manager",
            "--controller-manager-timeout", "120",
        ],
        output="screen",
    )

    # 4. move_group, RViz and the motion planner
    move_group = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config.to_dict(), use_sim_time],
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2_moveit",
        output="log",
        arguments=["-d", os.path.join(pkg_moveit_config, "config", "moveit.rviz")],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
            use_sim_time,
        ],
        condition=IfCondition(launch_rviz),
    )

    motion_planner = Node(
        package="sorting_arm_motion_planner",
        executable="motion_planner",
        output="screen",
        emulate_tty=True,
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
            use_sim_time,
        ],
    )

    # 5. Planning scene (table + obstacles from the randomized world)
    planning_scene_setup = Node(
        package="sorting_arm_motion_planner",
        executable="planning_scene_setup",
        output="screen",
        parameters=[use_sim_time],
    )

    # 6. Object detector
    object_detector = Node(
        package="sorting_arm_gazebo",
        executable="object_detector.py",
        output="screen",
        parameters=[{"show_image": show_camera}, use_sim_time],
    )

    # 7. State machine
    state_machine = Node(
        package="sorting_arm",
        executable="state_machine",
        output="screen",
        parameters=[use_sim_time],
    )

    # The planner prints PLANNER_READY_MSG once its MoveGroupInterface is connected to
    # move_group, which is when the planning scene can be populated.
    planner_ready_triggered = []

    def on_planner_output(event):
        if planner_ready_triggered or PLANNER_READY_MSG not in event.text:
            return None
        planner_ready_triggered.append(True)
        return [planning_scene_setup]

    return LaunchDescription([
        DeclareLaunchArgument(
            "gazebo_gui", default_value="true",
            description="Start the Gazebo GUI; false runs the server only (gz sim -s).",
        ),
        DeclareLaunchArgument(
            "headless_rendering", default_value="false",
            description="Render the camera with EGL (gz sim --headless-rendering) for machines "
                        "without a display. Needs a GPU/EGL driver.",
        ),
        DeclareLaunchArgument(
            "launch_rviz", default_value=gazebo_gui,
            description="Start MoveIt's RViz (defaults to gazebo_gui).",
        ),
        DeclareLaunchArgument(
            "show_camera", default_value=gazebo_gui,
            description="Show the object detector's cv2 window (defaults to gazebo_gui).",
        ),
        AppendEnvironmentVariable(
            name="GZ_SIM_RESOURCE_PATH", value=os.path.join(pkg_gazebo, "models")
        ),
        randomize_world,
        RegisterEventHandler(OnProcessExit(
            target_action=randomize_world,
            on_exit=on_success(
                "World randomization",
                [gazebo, robot_state_publisher, clock_bridge, camera_bridge, spawn_robot],
            ),
        )),
        RegisterEventHandler(OnProcessExit(
            target_action=spawn_robot,
            on_exit=on_success("Spawning the robot", [controller_spawner]),
        )),
        RegisterEventHandler(OnProcessExit(
            target_action=controller_spawner,
            on_exit=on_success("Controller spawning", [move_group, rviz, motion_planner]),
        )),
        RegisterEventHandler(OnProcessIO(
            target_action=motion_planner,
            on_stdout=on_planner_output,
            on_stderr=on_planner_output,
        )),
        RegisterEventHandler(OnProcessExit(
            target_action=planning_scene_setup,
            on_exit=[object_detector],
        )),
        RegisterEventHandler(OnProcessStart(
            target_action=object_detector,
            on_start=[state_machine],
        )),
    ])
