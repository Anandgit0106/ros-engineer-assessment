#ifndef NAV2_SIMPLE_CONTROLLER__SIMPLE_CONTROLLER_HPP_
#define NAV2_SIMPLE_CONTROLLER__SIMPLE_CONTROLLER_HPP_

#include <string>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "geometry_msgs/msg/twist_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include "nav_msgs/msg/path.hpp"

#include "nav2_core/controller.hpp"

namespace nav2_simple_controller
{

class SimpleController : public nav2_core::Controller
{
public:
  SimpleController() = default;
  ~SimpleController() override = default;

  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  void cleanup() override;

  void activate() override;

  void deactivate() override;

  void setPlan(
    const nav_msgs::msg::Path & path) override;

  geometry_msgs::msg::TwistStamped
  computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & velocity,
    nav2_core::GoalChecker * goal_checker) override;

  void setSpeedLimit(
    const double & speed_limit,
    const bool & percentage) override;

private:

  nav_msgs::msg::Path global_plan_;

  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;

  std::string plugin_name_;

  double kp_;
  double linear_velocity_;
};

}  // namespace nav2_simple_controller

#endif