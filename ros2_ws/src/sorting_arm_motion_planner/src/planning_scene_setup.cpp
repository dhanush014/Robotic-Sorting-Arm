#include <chrono>
#include <memory>
#include <thread>

#include <rclcpp/rclcpp.hpp>

#include <moveit/planning_scene_interface/planning_scene_interface.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<rclcpp::Node>("planning_scene_setup");

  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  moveit_msgs::msg::CollisionObject table;
  table.header.frame_id = "world";
  table.id = "sorting_table";

  shape_msgs::msg::SolidPrimitive primitive;
  primitive.type = shape_msgs::msg::SolidPrimitive::BOX;

  primitive.dimensions = {
    1.5,   // X
    1.0,   // Y
    0.05   // Z
  };

  geometry_msgs::msg::Pose table_pose;
  table_pose.orientation.w = 1.0;

  table_pose.position.x = 0.0;
  table_pose.position.y = 0.0;
  table_pose.position.z = 0.75;

  table.primitives.push_back(primitive);
  table.primitive_poses.push_back(table_pose);
  table.operation = moveit_msgs::msg::CollisionObject::ADD;

  RCLCPP_INFO(node->get_logger(), "Adding sorting table to MoveIt planning scene...");

  planning_scene_interface.applyCollisionObject(table);

  RCLCPP_INFO(node->get_logger(), "Table added.");

  // Give MoveIt/RViz time to receive it.
  std::this_thread::sleep_for(std::chrono::seconds(2));

  rclcpp::shutdown();
  return 0;
}
