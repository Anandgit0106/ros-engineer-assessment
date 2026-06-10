#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "agv_charging/action/go_to_charger.hpp"

using GoToCharger = agv_charging::action::GoToCharger;
using GoalHandle  = rclcpp_action::ClientGoalHandle<GoToCharger>;

class ChargingActionClient : public rclcpp::Node
{
public:
  ChargingActionClient() : Node("charging_action_client")
  {
    client_ = rclcpp_action::create_client<GoToCharger>(this, "go_to_charger_act");
  }

  void send_goal()
  {
    if (!client_->wait_for_action_server(std::chrono::seconds(5))) {
      RCLCPP_ERROR(this->get_logger(), "Action server not available!");
      return;
    }

    auto goal_msg = GoToCharger::Goal();
    goal_msg.go = true;

    auto send_goal_options = rclcpp_action::Client<GoToCharger>::SendGoalOptions();

    // Called when server accepts/rejects the goal
    send_goal_options.goal_response_callback =
      [this](const GoalHandle::SharedPtr & goal_handle) {
        if (!goal_handle) {
          RCLCPP_ERROR(this->get_logger(), "Goal rejected by server.");
        } else {
          RCLCPP_INFO(this->get_logger(), "Goal accepted — receiving feedback...");
        }
      };

    // Called every time server publishes feedback
    send_goal_options.feedback_callback =
      [this](GoalHandle::SharedPtr,
             const std::shared_ptr<const GoToCharger::Feedback> feedback) {
        RCLCPP_INFO(this->get_logger(),
          "[feedback] Distance remaining: %.1f m", feedback->distance_remaining);
      };

    // Called when the goal finishes
    send_goal_options.result_callback =
      [this](const GoalHandle::WrappedResult & wrapped_result) {
        switch (wrapped_result.code) {
          case rclcpp_action::ResultCode::SUCCEEDED:
            RCLCPP_INFO(this->get_logger(),
              "✔ Result: %s Travel time: %.1fs",
              wrapped_result.result->message.c_str(),
              wrapped_result.result->travel_time);
            break;
          case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_WARN(this->get_logger(), "Goal was cancelled.");
            break;
          default:
            RCLCPP_ERROR(this->get_logger(), "Goal failed.");
        }
        rclcpp::shutdown();
      };

    client_->async_send_goal(goal_msg, send_goal_options);
  }

private:
  rclcpp_action::Client<GoToCharger>::SharedPtr client_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto client_node = std::make_shared<ChargingActionClient>();
  client_node->send_goal();
  rclcpp::spin(client_node);
  return 0;
}