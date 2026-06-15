#ifndef NAV2_SIMPLE_CONTROLLER__SIMPLE_CONTROLLER_HPP_
#define NAV2_SIMPLE_CONTROLLER__SIMPLE_CONTROLLER_HPP_

#include <string>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "nav2_core/controller.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"

#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"

#include "tf2_ros/buffer.h"

namespace nav2_simple_controller
{

class SimpleController : public nav2_core::Controller
{
public:
  SimpleController() = default;
  ~SimpleController() override = default;

  // ── Lifecycle methods ──────────────────────────────────────────────────────
  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  void cleanup() override;
  void activate() override;
  void deactivate() override;

  // ── Core controller methods ────────────────────────────────────────────────
  void setPlan(const nav_msgs::msg::Path & path) override;

  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & velocity,
    nav2_core::GoalChecker * goal_checker) override;

  void setSpeedLimit(
    const double & speed_limit,
    const bool & percentage) override;

private:
  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  std::string plugin_name_;
  nav_msgs::msg::Path global_plan_;
  double kp_;
  double linear_velocity_;
  double lookahead_dist_;                              
};

}  // namespace nav2_simple_controller

#endif  // NAV2_SIMPLE_CONTROLLER__SIMPLE_CONTROLLER_HPP_
