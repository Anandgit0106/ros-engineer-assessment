#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "agv_charging/action/go_to_charger.hpp"
#include <thread>
#include <chrono>

using GoToCharger = agv_charging::action::GoToCharger;
using GoalHandle  = rclcpp_action::ServerGoalHandle<GoToCharger>;

class ChargingActionServer : public rclcpp::Node
{
public:
  ChargingActionServer() : Node("charging_action_server")
  {
    server_ = rclcpp_action::create_server<GoToCharger>(
      this,
      "go_to_charger_act",
      // Goal callback
      [](const rclcpp_action::GoalUUID &, std::shared_ptr<const GoToCharger::Goal>) {
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
      },
      // Cancel callback
      [](const std::shared_ptr<GoalHandle>) {
        return rclcpp_action::CancelResponse::ACCEPT;
      },
      // Accepted callback — runs the goal
      [this](const std::shared_ptr<GoalHandle> goal_handle) {
        std::thread([this, goal_handle]() { execute(goal_handle); }).detach();
      }
    );
    RCLCPP_INFO(this->get_logger(), "Action server ready.");
  }

private:
  rclcpp_action::Server<GoToCharger>::SharedPtr server_;

  void execute(const std::shared_ptr<GoalHandle> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Executing 30s charging trip...");

    auto feedback = std::make_shared<GoToCharger::Feedback>();
    auto result   = std::make_shared<GoToCharger::Result>();

    const float total_distance = 30.0f;   // 30 m at 1 m/s = 30 s
    float distance = total_distance;

    rclcpp::Rate rate(1);  // 1 Hz feedback

    for (int i = 0; i < 30; ++i) {
      if (goal_handle->is_canceling()) {
        result->success = false;
        result->message = "Cancelled";
        goal_handle->canceled(result);
        return;
      }

      distance -= 1.0f;
      feedback->distance_remaining = distance;
      goal_handle->publish_feedback(feedback);
      RCLCPP_INFO(this->get_logger(), "Distance remaining: %.1f m", distance);
      rate.sleep();
    }

    result->success     = true;
    result->travel_time = 30.0f;
    result->message     = "Arrived at charging station!";
    goal_handle->succeed(result);
    RCLCPP_INFO(this->get_logger(), "Goal succeeded.");
  }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ChargingActionServer>());
  rclcpp::shutdown();
  return 0;
}