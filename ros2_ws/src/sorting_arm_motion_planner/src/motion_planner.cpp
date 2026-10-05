#include <memory>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include "sorting_arm_motion_planner/srv/move_to_target.hpp"

bool move_cartesian_z(
  moveit::planning_interface::MoveGroupInterface &move_group,
  double delta_z,
  const rclcpp::Logger &logger)
{
  move_group.setStartStateToCurrentState();

  auto current_pose = move_group.getCurrentPose().pose;
  auto target_pose = current_pose;
  target_pose.position.z += delta_z;

  RCLCPP_INFO(
    logger,
    "Current: x=%.3f y=%.3f z=%.3f",
    current_pose.position.x,
    current_pose.position.y,
    current_pose.position.z
  );

  RCLCPP_INFO(
    logger,
    "Target:  x=%.3f y=%.3f z=%.3f",
    target_pose.position.x,
    target_pose.position.y,
    target_pose.position.z
  );

  std::vector<geometry_msgs::msg::Pose> waypoints{target_pose};
  moveit_msgs::msg::RobotTrajectory trajectory;

  double fraction = move_group.computeCartesianPath(
    waypoints,
    0.005,
    trajectory,
    true
  );

  RCLCPP_INFO(
    logger,
    "Cartesian path achieved %.1f%%",
    fraction * 100.0
  );

  if (fraction < 0.95) {
    RCLCPP_ERROR(logger, "Could not compute enough of Cartesian path");
    return false;
  }

  auto result = move_group.execute(trajectory);

  if (!result) {
    RCLCPP_ERROR(logger, "Cartesian execution failed");
    return false;
  }

  RCLCPP_INFO(logger, "Cartesian execution succeeded");
  return true;
}

bool move_to_pose(
  moveit::planning_interface::MoveGroupInterface &move_group,
  const geometry_msgs::msg::Pose &target_pose,
  const rclcpp::Logger &logger)
{
  move_group.setStartStateToCurrentState();
  move_group.setPoseTarget(target_pose);

  RCLCPP_INFO(logger, "Planning to target pose...");

  moveit::planning_interface::MoveGroupInterface::Plan plan;
  moveit::core::MoveItErrorCode plan_result = move_group.plan(plan);
  if (plan_result != moveit::core::MoveItErrorCode::SUCCESS) {
    RCLCPP_ERROR(logger, "Could not plan path to target pose (error code: %d)", plan_result.val);
    return false;
  }

  RCLCPP_INFO(logger, "Plan found, executing...");

  moveit::core::MoveItErrorCode exec_result = move_group.execute(plan);
  if (exec_result != moveit::core::MoveItErrorCode::SUCCESS) {
    RCLCPP_ERROR(logger, "Execution failed");
    return false;
  }

  RCLCPP_INFO(logger, "Execution succeeded");
  return true;
}

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<rclcpp::Node>(
    "motion_planner",
    rclcpp::NodeOptions()
      .automatically_declare_parameters_from_overrides(true)
  );

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);

  std::thread spinner([&executor]() {
    executor.spin();
  });

  auto move_group =
    moveit::planning_interface::MoveGroupInterface(node, "arm");

  RCLCPP_INFO(
    node->get_logger(),
    "Planning frame: %s",
    move_group.getPlanningFrame().c_str()
  );

  RCLCPP_INFO(
    node->get_logger(),
    "End effector: %s",
    move_group.getEndEffectorLink().c_str()
  );

  RCLCPP_INFO(
    node->get_logger(),
    "Pose reference frame: %s",
    move_group.getPoseReferenceFrame().c_str()
  );

  auto service_callback_group = node->create_callback_group(
    rclcpp::CallbackGroupType::Reentrant);

  auto service = node->create_service<sorting_arm_motion_planner::srv::MoveToTarget>(
    "/move_to_pose",
    [&move_group, &node](
      const std::shared_ptr<sorting_arm_motion_planner::srv::MoveToTarget::Request> request,
      std::shared_ptr<sorting_arm_motion_planner::srv::MoveToTarget::Response> response)
    {
      response->success = move_to_pose(move_group, request->target_pose, node->get_logger());
    },
    rclcpp::ServicesQoS(),
    service_callback_group);

  RCLCPP_INFO(node->get_logger(), "Motion planner service /move_to_pose started");

  spinner.join();

  rclcpp::shutdown();

  return 0;
}
