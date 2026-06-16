#include <chrono>
#include <memory>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/transform_broadcaster.h"

using namespace std::chrono_literals;


class TFManagerNode : public rclcpp::Node
{
public:
  TFManagerNode()
  : Node("tf_manager_node"),
    base_x_(0.0), base_y_(0.0), base_yaw_(0.0),
    odom_x_(0.0), odom_y_(0.0), odom_yaw_(0.0)
  {
    broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    // Both transforms start at origin — all frames overlap
    map_to_odom_.header.frame_id  = "map";
    map_to_odom_.child_frame_id   = "odom";
    odom_to_base_.header.frame_id = "odom";
    odom_to_base_.child_frame_id  = "base_link";

    setTF(map_to_odom_,  0.0, 0.0, 0.0);
    setTF(odom_to_base_, 0.0, 0.0, 0.0);

    goal_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/goal_pose", 10,
      std::bind(&TFManagerNode::goalCallback, this, std::placeholders::_1));

    // Broadcast at 10 Hz to keep TF tree alive
    timer_ = this->create_wall_timer(
      100ms, std::bind(&TFManagerNode::broadcastTransforms, this));

    RCLCPP_INFO(this->get_logger(), "TF Manager Node started.");
    RCLCPP_INFO(this->get_logger(), "map, odom, base_link all at origin.");
    RCLCPP_INFO(this->get_logger(), "Send a 2D Goal Pose from RViz.");
  }

private:

  
  void goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    
    base_x_   = msg->pose.position.x;
    base_y_   = msg->pose.position.y;
    base_yaw_ = quaternionToYaw(msg->pose.orientation);

    const double offset_x   =  0.50;   
    const double offset_y   =  0.16;   
    const double offset_yaw =  0.05;   

    odom_x_   = base_x_   + offset_x;
    odom_y_   = base_y_   + offset_y;
    odom_yaw_ = base_yaw_ + offset_yaw;

    
    setTF(map_to_odom_, odom_x_, odom_y_, odom_yaw_);

    
    double rel_x         = base_x_ - odom_x_;
    double rel_y         = base_y_ - odom_y_;
    double cos_o         = std::cos(-odom_yaw_);
    double sin_o         = std::sin(-odom_yaw_);
    double b_in_odom_x   =  cos_o * rel_x - sin_o * rel_y;
    double b_in_odom_y   =  sin_o * rel_x + cos_o * rel_y;
    double b_in_odom_yaw =  base_yaw_ - odom_yaw_;
    setTF(odom_to_base_, b_in_odom_x, b_in_odom_y, b_in_odom_yaw);

    
  }

  // ── Broadcast transforms at 10 Hz ────────────────────────────────────────
  void broadcastTransforms()
  {
    auto now = this->get_clock()->now();
    map_to_odom_.header.stamp  = now;
    odom_to_base_.header.stamp = now;
    broadcaster_->sendTransform(map_to_odom_);
    broadcaster_->sendTransform(odom_to_base_);
  }

 
  void setTF(geometry_msgs::msg::TransformStamped & tf,
             double x, double y, double yaw)
  {
    tf.transform.translation.x = x;
    tf.transform.translation.y = y;
    tf.transform.translation.z = 0.0;
    tf.transform.rotation.x    = 0.0;
    tf.transform.rotation.y    = 0.0;
    tf.transform.rotation.z    = std::sin(yaw / 2.0);
    tf.transform.rotation.w    = std::cos(yaw / 2.0);
  }

  // ── Helper: quaternion to yaw ─────────────────────────────────────────────
  double quaternionToYaw(const geometry_msgs::msg::Quaternion & q)
  {
    return std::atan2(
      2.0 * (q.w * q.z + q.x * q.y),
      1.0 - 2.0 * (q.y * q.y + q.z * q.z));
  }

  // ── Members ───────────────────────────────────────────────────────────────
  std::shared_ptr<tf2_ros::TransformBroadcaster>                   broadcaster_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  rclcpp::TimerBase::SharedPtr                                      timer_;

  geometry_msgs::msg::TransformStamped map_to_odom_;
  geometry_msgs::msg::TransformStamped odom_to_base_;

  double base_x_, base_y_, base_yaw_;   
  double odom_x_, odom_y_, odom_yaw_;   
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TFManagerNode>());
  rclcpp::shutdown();
  return 0;
}
