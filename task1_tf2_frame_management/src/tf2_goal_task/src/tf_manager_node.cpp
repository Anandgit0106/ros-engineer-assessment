#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"

#include "tf2_ros/transform_broadcaster.h"

using namespace std::chrono_literals;

class TFManagerNode : public rclcpp::Node
{
public:
    TFManagerNode()
    : Node("tf_manager_node")
    {
        broadcaster_ =
            std::make_shared<tf2_ros::TransformBroadcaster>(this);

        initializeTransforms();

        goal_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/goal_pose",
            10,
            std::bind(&TFManagerNode::goalCallback, this, std::placeholders::_1));

        timer_ = this->create_wall_timer(
            100ms,
            std::bind(&TFManagerNode::broadcastTransforms, this));

        RCLCPP_INFO(
            this->get_logger(),
            "TF Manager Node Started");
    }

private:

    void initializeTransforms()
    {
        // -----------------------------
        // map -> odom
        // -----------------------------
        map_to_odom_.header.frame_id = "map";
        map_to_odom_.child_frame_id = "odom";

        map_to_odom_.transform.translation.x = 0.0;
        map_to_odom_.transform.translation.y = 0.0;
        map_to_odom_.transform.translation.z = 0.0;

        map_to_odom_.transform.rotation.x = 0.0;
        map_to_odom_.transform.rotation.y = 0.0;
        map_to_odom_.transform.rotation.z = 0.0;
        map_to_odom_.transform.rotation.w = 1.0;

        // -----------------------------
        // odom -> base_link
        // -----------------------------
        odom_to_base_.header.frame_id = "odom";
        odom_to_base_.child_frame_id = "base_link";

        odom_to_base_.transform.translation.x = 0.0;
        odom_to_base_.transform.translation.y = 0.0;
        odom_to_base_.transform.translation.z = 0.0;

        odom_to_base_.transform.rotation.x = 0.0;
        odom_to_base_.transform.rotation.y = 0.0;
        odom_to_base_.transform.rotation.z = 0.0;
        odom_to_base_.transform.rotation.w = 1.0;
    }

    void goalCallback(
        const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        map_to_odom_.transform.translation.x =
            msg->pose.position.x;

        map_to_odom_.transform.translation.y =
            msg->pose.position.y;

        map_to_odom_.transform.translation.z = 0.0;

        // Use quaternion directly from RViz goal pose
        map_to_odom_.transform.rotation =
            msg->pose.orientation;

        RCLCPP_INFO(
            this->get_logger(),
            "Received Goal: x=%.2f y=%.2f",
            msg->pose.position.x,
            msg->pose.position.y);
    }

    void broadcastTransforms()
    {
        auto now = this->get_clock()->now();

        map_to_odom_.header.stamp = now;
        odom_to_base_.header.stamp = now;

        broadcaster_->sendTransform(map_to_odom_);
        broadcaster_->sendTransform(odom_to_base_);
    }

    std::shared_ptr<tf2_ros::TransformBroadcaster> broadcaster_;

    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;

    rclcpp::TimerBase::SharedPtr timer_;

    geometry_msgs::msg::TransformStamped map_to_odom_;
    geometry_msgs::msg::TransformStamped odom_to_base_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<TFManagerNode>();

    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}