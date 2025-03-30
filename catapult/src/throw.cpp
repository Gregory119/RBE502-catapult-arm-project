#include <memory>

#include <numbers>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <moveit_visual_tools/moveit_visual_tools.h>

#include <Eigen/Dense>


int main(int argc, char * argv[])
{
  // Initialize ROS and create the Node
  rclcpp::init(argc, argv);
  auto const node = std::make_shared<rclcpp::Node>(
    "throw",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
  );

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread([&executor]() { executor.spin(); }).detach();

  // Create a ROS logger
  auto const logger = rclcpp::get_logger("throw");

  // Create the MoveIt MoveGroup Interface
  using moveit::planning_interface::MoveGroupInterface;
  auto move_group = MoveGroupInterface(node, "ur_manipulator");

  // Construct and initialize MoveItVisualTools
  namespace rvt = rviz_visual_tools;
  auto visual_tools = moveit_visual_tools::MoveItVisualTools{
      node, "base_link", rvt::RVIZ_MARKER_TOPIC,
      move_group.getRobotModel()};
  visual_tools.deleteAllMarkers();
  visual_tools.loadRemoteControl();
  visual_tools.trigger();
  
  // Set a start pose
  using namespace std::numbers;
  Eigen::Isometry3d mat_end_pose = Eigen::Translation3d(Eigen::Vector3d(0.0, -0.6, 0.8)) *
      Eigen::AngleAxisd(pi, Eigen::Vector3d(0,0,1.0)) * Eigen::AngleAxisd(-pi/6, Eigen::Vector3d(0,1.0,0));
  Eigen::Isometry3d mat_start_pose = mat_end_pose * Eigen::Translation3d(Eigen::Vector3d(-0.6,0,0));
  
  geometry_msgs::msg::Pose start_pose = tf2::toMsg(mat_start_pose);
  geometry_msgs::msg::Pose end_pose = tf2::toMsg(mat_end_pose);
    
  move_group.setPoseTarget(start_pose);

  visual_tools.publishAxisLabeled(start_pose, "start_pose");
  visual_tools.publishAxisLabeled(end_pose, "end_pose");
  visual_tools.trigger();
  
  // move to start pose

  // Create a plan
  auto const plan_fn = [&move_group]{
      moveit::planning_interface::MoveGroupInterface::Plan msg;
      auto const ok = static_cast<bool>(move_group.plan(msg));
      return std::make_pair(ok, msg);
  };
  const auto [success, plan] = plan_fn();

  // Execute the plan
  if(success) {
      move_group.execute(plan);
  } else {
      RCLCPP_ERROR(logger, "Planning failed!");
  }

  // cartesian plan between start and end pose
  std::vector<geometry_msgs::msg::Pose> waypoints;
  waypoints.push_back(start_pose);
  waypoints.push_back(end_pose);

  const double eef_step = 0.01;
  const double jump_threshold = 5.0;
  moveit_msgs::msg::RobotTrajectory trajectory;
  double fraction = move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
  RCLCPP_INFO(logger, "Visualizing cartesion path (%.2f%% achieved)", fraction * 100.0);

  //visual_tools.deleteAllMarkers();
  visual_tools.publishPath(waypoints, rvt::LIME_GREEN, rvt::SMALL);
  visual_tools.trigger();

  // execute trajectory
  move_group.execute(trajectory);

  // Shutdown ROS
  rclcpp::shutdown();
  return 0;
}
