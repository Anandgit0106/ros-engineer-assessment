
#include "nav2_simple_controller/simple_controller.hpp"

#include <cmath>
#include <tf2/utils.h>

#include <pluginlib/class_list_macros.hpp>

namespace nav2_simple_controller
{

void SimpleController::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name,
  std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
    node_ = parent.lock();

  plugin_name_ = name;

  kp_ = 1.0;
  linear_velocity_ = 0.2;

  RCLCPP_INFO(
    node_->get_logger(),
    "SimpleController configured");
}

void SimpleController::cleanup()
{
 global_plan_.poses.clear();

  RCLCPP_INFO(
    node_->get_logger(),
    "SimpleController cleaned up");

}

void SimpleController::activate()
{
     RCLCPP_INFO(
    node_->get_logger(),
    "SimpleController activated");
}

void SimpleController::deactivate()
{
    RCLCPP_INFO(
    node_->get_logger(),
    "SimpleController deactivated");
}

void SimpleController::setPlan(
  const nav_msgs::msg::Path & path)
{
     global_plan_ = path;

  RCLCPP_INFO(
    node_->get_logger(),
    "Received plan with %zu poses",
    global_plan_.poses.size());
}

geometry_msgs::msg::TwistStamped
SimpleController::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & pose,
  const geometry_msgs::msg::Twist & velocity,
  nav2_core::GoalChecker * goal_checker)
{
  geometry_msgs::msg::TwistStamped cmd;
   cmd.header.stamp = node_->now();
  cmd.header.frame_id = "base_link";

  if (global_plan_.poses.empty()) {
    return cmd;
  }

  const auto & target = global_plan_.poses.front();

  double robot_x = pose.pose.position.x;
  double robot_y = pose.pose.position.y;

  double target_x = target.pose.position.x;
  double target_y = target.pose.position.y;

  double dx = target_x - robot_x;
  double dy = target_y - robot_y;

  double desired_heading = std::atan2(dy, dx);

  double robot_yaw =
  tf2::getYaw(pose.pose.orientation);
  double heading_error =
    desired_heading - robot_yaw;

  cmd.twist.linear.x =
    linear_velocity_;

  cmd.twist.angular.z =
    kp_ * heading_error;
  return cmd;
}

void SimpleController::setSpeedLimit(
  const double & speed_limit,
  const bool & percentage)
{
   (void)speed_limit;
  (void)percentage;
}

}  // namespace nav2_simple_controller

PLUGINLIB_EXPORT_CLASS(
  nav2_simple_controller::SimpleController,
  nav2_core::Controller)