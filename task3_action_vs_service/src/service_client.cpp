#include "rclcpp/rclcpp.hpp"
#include "agv_charging/srv/go_to_charger.hpp"
#include <chrono>

using GoToCharger = agv_charging::srv::GoToCharger;
using namespace std::chrono_literals;

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("charging_service_client");
  auto client = node->create_client<GoToCharger>("go_to_charger_srv");

  RCLCPP_INFO(node->get_logger(), "Requesting charging station trip via SERVICE...");

  // Wait for server to be available
  while (!client->wait_for_service(1s)) {
    RCLCPP_WARN(node->get_logger(), "Waiting for service server...");
  }

  auto request = std::make_shared<GoToCharger::Request>();
  request->go = true;

  RCLCPP_INFO(node->get_logger(), "Waiting for response (5s timeout)...");

  auto future = client->async_send_request(request);

  // 5-second timeout
  auto status = rclcpp::spin_until_future_complete(node, future, 5s);

  if (status == rclcpp::FutureReturnCode::SUCCESS) {
    auto result = future.get();
    RCLCPP_INFO(node->get_logger(), "✔ Result: %s Travel time: %.1fs",
      result->message.c_str(), result->travel_time);
  } else if (status == rclcpp::FutureReturnCode::TIMEOUT) {
    RCLCPP_ERROR(node->get_logger(),
      "✘ TIMED OUT — service did not respond within 5 seconds!");
  } else {
    RCLCPP_ERROR(node->get_logger(), "✘ Service call interrupted/failed.");
  }

  rclcpp::shutdown();
  return 0;
}