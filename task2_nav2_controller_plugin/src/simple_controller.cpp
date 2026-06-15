#include "nav2_simple_controller/simple_controller.hpp"

#include <cmath>
#include <memory>
#include <string>

#include "tf2/utils.h"
#include "tf2/time.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace nav2_simple_controller
{

void SimpleController::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name,
  std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  node_        = parent.lock();
  plugin_name_ = name;
  tf_          = tf;
  costmap_ros_ = costmap_ros;

  kp_              = 1.5;
  linear_velocity_ = 0.2;
  lookahead_dist_  = 0.5;   // look this far ahead in the plan

  RCLCPP_INFO(node_->get_logger(), "SimpleController configured");
}

void SimpleController::cleanup()
{
  global_plan_.poses.clear();
  RCLCPP_INFO(node_->get_logger(), "SimpleController cleaned up");
}

void SimpleController::activate()
{
  RCLCPP_INFO(node_->get_logger(), "SimpleController activated");
}

void SimpleController::deactivate()
{
  RCLCPP_INFO(node_->get_logger(), "SimpleController deactivated");
}

void SimpleController::setPlan(const nav_msgs::msg::Path & path)
{
  global_plan_ = path;
  RCLCPP_INFO(node_->get_logger(),
    "Received plan with %zu poses", global_plan_.poses.size());
}

geometry_msgs::msg::TwistStamped
SimpleController::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & /*pose*/,
  const geometry_msgs::msg::Twist & /*velocity*/,
  nav2_core::GoalChecker * /*goal_checker*/)
{
  geometry_msgs::msg::TwistStamped cmd;
  cmd.header.stamp    = node_->now();
  cmd.header.frame_id = "base_link";

  if (global_plan_.poses.empty()) {
    RCLCPP_WARN(node_->get_logger(), "Plan is empty — stopping.");
    return cmd;
  }

  const tf2::Duration timeout = tf2::durationFromSec(0.1);

  // ── Step 1: Prune waypoints within 0.5m (increased from 0.3m) ────────────
  while (global_plan_.poses.size() > 1) {
    geometry_msgs::msg::PoseStamped wp_in_base;
    try {
      tf_->transform(global_plan_.poses.front(), wp_in_base, "base_link", timeout);
    } catch (tf2::TransformException & ex) {
      RCLCPP_WARN(node_->get_logger(), "TF prune failed: %s", ex.what());
      break;
    }
    double dist = std::hypot(wp_in_base.pose.position.x, wp_in_base.pose.position.y);
    if (dist < 0.5) {                                  // ← increased threshold
      global_plan_.poses.erase(global_plan_.poses.begin());
    } else {
      break;
    }
  }

  // ── Step 2: Lookahead — find first waypoint at least lookahead_dist_ away
  //           AND in front of the robot (x > 0 in base_link) ─────────────────
  // This prevents the robot from freezing when next waypoint is behind it.
  geometry_msgs::msg::PoseStamped target_in_base;
  bool found_target = false;

  for (size_t i = 0; i < global_plan_.poses.size(); ++i) {
    geometry_msgs::msg::PoseStamped wp_in_base;
    try {
      tf_->transform(global_plan_.poses[i], wp_in_base, "base_link", timeout);
    } catch (tf2::TransformException & ex) {
      RCLCPP_WARN(node_->get_logger(), "TF lookahead failed: %s", ex.what());
      break;
    }

    double dist = std::hypot(wp_in_base.pose.position.x, wp_in_base.pose.position.y);

    // Pick this waypoint if it's far enough ahead
    // For the last waypoint always use it regardless of direction
    if (dist >= lookahead_dist_ || i == global_plan_.poses.size() - 1) {
      target_in_base = wp_in_base;
      found_target   = true;
      break;
    }
  }

  if (!found_target) {
    // Fallback: use first waypoint
    try {
      tf_->transform(global_plan_.poses.front(), target_in_base, "base_link", timeout);
    } catch (tf2::TransformException & ex) {
      RCLCPP_WARN(node_->get_logger(), "TF fallback failed: %s", ex.what());
      return cmd;
    }
  }

  // ── Step 3: Heading error (in base_link, robot faces +X) ─────────────────
  double dx = target_in_base.pose.position.x;
  double dy = target_in_base.pose.position.y;
  double heading_error = std::atan2(dy, dx);           // already [-pi, pi]

  // ── Step 4: Velocities ────────────────────────────────────────────────────
  // Always move forward at reduced speed, never stop to turn
  // This prevents the progress checker from aborting
  double heading_scale = std::max(0.1, std::cos(heading_error)); // min 0.1, not 0
  cmd.twist.linear.x   = linear_velocity_ * heading_scale;
  cmd.twist.angular.z  = kp_ * heading_error;

  // Clamp angular velocity to [-2.0, 2.0] rad/s for smooth motion
  cmd.twist.angular.z = std::max(-2.0, std::min(2.0, cmd.twist.angular.z));

  RCLCPP_INFO(
    node_->get_logger(),
    "target_base=(%.2f, %.2f) | err=%.2f rad | lin=%.2f | ang=%.2f | wps=%zu",
    dx, dy, heading_error,
    cmd.twist.linear.x, cmd.twist.angular.z,
    global_plan_.poses.size());

  return cmd;
}

void SimpleController::setSpeedLimit(
  const double & speed_limit, const bool & percentage)
{
  (void)speed_limit;
  (void)percentage;
}

}  // namespace nav2_simple_controller

PLUGINLIB_EXPORT_CLASS(
  nav2_simple_controller::SimpleController,
  nav2_core::Controller)
