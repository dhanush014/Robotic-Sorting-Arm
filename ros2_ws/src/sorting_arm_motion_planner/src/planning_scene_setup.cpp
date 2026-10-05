#include <chrono>
#include <fstream>
#include <memory>
#include <sstream>
#include <thread>
#include <tuple>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include <moveit/planning_scene_interface/planning_scene_interface.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>

void add_obstacles_from_sdf(
  moveit::planning_interface::PlanningSceneInterface &psi,
  const rclcpp::Logger &logger)
{
  std::ifstream file("/tmp/sorting_world_randomized.sdf");
  if (!file.is_open()) {
    RCLCPP_ERROR(logger, "Failed to open /tmp/sorting_world_randomized.sdf");
    return;
  }

  std::string line;
  int obstacle_count = 0;

  while (std::getline(file, line)) {
    if (line.find("<uri>model://obstacle_") == std::string::npos) {
      continue;
    }

    // Extract obstacle type (bottle or ball)
    size_t uri_start = line.find("model://") + 8;
    size_t uri_end = line.find("</uri>");
    std::string obstacle_type = line.substr(uri_start, uri_end - uri_start);

    // Skip to <pose> line (2 lines ahead: <name> then <pose>)
    std::getline(file, line); // <name>
    std::getline(file, line); // <pose>

    if (line.find("<pose>") == std::string::npos) {
      RCLCPP_WARN(logger, "Expected <pose> tag, skipping obstacle");
      continue;
    }

    // Extract pose: X Y Z 0 0 0
    size_t pose_start = line.find(">") + 1;
    size_t pose_end = line.find("</pose>");
    std::string pose_str = line.substr(pose_start, pose_end - pose_start);

    double x, y, z, roll, pitch, yaw;
    std::istringstream iss(pose_str);
    if (!(iss >> x >> y >> z >> roll >> pitch >> yaw)) {
      RCLCPP_WARN(logger, "Failed to parse pose");
      continue;
    }

    // Create collision object
    moveit_msgs::msg::CollisionObject obstacle;
    obstacle.header.frame_id = "world";
    obstacle.id = obstacle_type + "_" + std::to_string(obstacle_count);
    obstacle.operation = moveit_msgs::msg::CollisionObject::ADD;

    shape_msgs::msg::SolidPrimitive primitive;
    geometry_msgs::msg::Pose obstacle_pose;
    obstacle_pose.orientation.w = 1.0;
    obstacle_pose.position.x = x;
    obstacle_pose.position.y = y;

    if (obstacle_type == "obstacle_bottle") {
      primitive.type = shape_msgs::msg::SolidPrimitive::CYLINDER;
      primitive.dimensions = {0.15, 0.065}; // height, radius (with safety margin)
      obstacle_pose.position.z = z + 0.075; // center offset
    } else if (obstacle_type == "obstacle_ball") {
      primitive.type = shape_msgs::msg::SolidPrimitive::SPHERE;
      primitive.dimensions = {0.07}; // radius (with safety margin)
      obstacle_pose.position.z = z + 0.05; // center offset
    } else {
      RCLCPP_WARN(logger, "Unknown obstacle type: %s", obstacle_type.c_str());
      continue;
    }

    obstacle.primitives.push_back(primitive);
    obstacle.primitive_poses.push_back(obstacle_pose);

    psi.applyCollisionObject(obstacle);
    obstacle_count++;
  }

  if (obstacle_count == 0) {
    RCLCPP_WARN(logger, "No obstacles found in SDF");
  } else {
    RCLCPP_INFO(logger, "Added %d obstacles to MoveIt planning scene", obstacle_count);
  }

  file.close();
}

void add_bins(
  moveit::planning_interface::PlanningSceneInterface &psi,
  const rclcpp::Logger &logger)
{
  // Bin poses from sorting_arm_gazebo/worlds/sorting_world.sdf and box geometry from
  // models/{red,blue}_bin/model.sdf (both bins are identical): a 0.35 x 0.35 x 0.05 bottom
  // and four 0.25 m tall, 0.02 m thick walls, all relative to the bin origin on the ground.
  struct Box { double x, y, z, sx, sy, sz; };
  const std::vector<Box> parts = {
    {0.0, 0.0, 0.025, 0.35, 0.35, 0.05},     // bottom
    {0.0, 0.165, 0.125, 0.35, 0.02, 0.25},   // left wall
    {0.0, -0.165, 0.125, 0.35, 0.02, 0.25},  // right wall
    {0.165, 0.0, 0.125, 0.02, 0.35, 0.25},   // front wall
    {-0.165, 0.0, 0.125, 0.02, 0.35, 0.25},  // back wall
  };
  const std::vector<std::tuple<std::string, double, double>> bins = {
    {"red_bin", -0.5, -0.8},
    {"blue_bin", 0.5, -0.8},
  };

  for (const auto &[id, bx, by] : bins) {
    moveit_msgs::msg::CollisionObject bin;
    bin.header.frame_id = "world";
    bin.id = id;
    bin.pose.orientation.w = 1.0;
    bin.pose.position.x = bx;
    bin.pose.position.y = by;

    for (const auto &part : parts) {
      shape_msgs::msg::SolidPrimitive primitive;
      primitive.type = shape_msgs::msg::SolidPrimitive::BOX;
      primitive.dimensions = {part.sx, part.sy, part.sz};

      geometry_msgs::msg::Pose pose;
      pose.orientation.w = 1.0;
      pose.position.x = part.x;
      pose.position.y = part.y;
      pose.position.z = part.z;

      bin.primitives.push_back(primitive);
      bin.primitive_poses.push_back(pose);
    }
    bin.operation = moveit_msgs::msg::CollisionObject::ADD;
    psi.applyCollisionObject(bin);
  }

  RCLCPP_INFO(logger, "Added %zu bins to MoveIt planning scene", bins.size());
}

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
    1.5,    // X
    0.995,  // Y
    0.05    // Z
  };

  geometry_msgs::msg::Pose table_pose;
  table_pose.orientation.w = 1.0;

  table_pose.position.x = 0.0;
  table_pose.position.y = 0.0025;
  table_pose.position.z = 0.75;

  table.primitives.push_back(primitive);
  table.primitive_poses.push_back(table_pose);
  table.operation = moveit_msgs::msg::CollisionObject::ADD;

  RCLCPP_INFO(node->get_logger(), "Adding sorting table to MoveIt planning scene...");

  planning_scene_interface.applyCollisionObject(table);

  RCLCPP_INFO(node->get_logger(), "Table added.");

  RCLCPP_INFO(node->get_logger(), "Loading obstacles from Gazebo world...");
  add_obstacles_from_sdf(planning_scene_interface, node->get_logger());

  add_bins(planning_scene_interface, node->get_logger());

  // Give MoveIt/RViz time to receive all objects.
  std::this_thread::sleep_for(std::chrono::seconds(2));

  rclcpp::shutdown();
  return 0;
}
