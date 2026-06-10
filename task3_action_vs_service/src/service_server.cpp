#include "rclcpp/rclcpp.hpp"
#include "agv_charging/srv/go_to_charger.hpp"
#include <thread>
#include <chrono>

using GoToCharger = agv_charging::srv::GoToCharger;

class ChargingServiceServer : public rclcpp::Node
{
public:
  ChargingServiceServer() : Node("charging_service_server")
  {
    server_ = this->create_service<GoToCharger>(
      "go_to_charger_srv",
      [this](const std::shared_ptr<GoToCharger::Request> request,
             std::shared_ptr<GoToCharger::Response> response) {
        this->handle_request(request, response);
      });
    RCLCPP_INFO(this->get_logger(), "Service server ready — will take 30s to respond.");
  }

private:
  rclcpp::Service<GoToCharger>::SharedPtr server_;

  void handle_request(
    const std::shared_ptr<GoToCharger::Request> /*request*/,
    std::shared_ptr<GoToCharger::Response> response)
  {
    RCLCPP_INFO(this->get_logger(), "Received request. Simulating 30s travel...");
    std::this_thread::sleep_for(std::chrono::seconds(30));  // blocks the service
    response->success = true;
    response->travel_time = 30.0f;
    response->message = "Arrived at charging station!";
    RCLCPP_INFO(this->get_logger(), "Done — sending response.");
  }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ChargingServiceServer>());
  rclcpp::shutdown();
  return 0;
}