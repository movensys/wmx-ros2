#include "rclcpp/rclcpp.hpp"
#include "wmx_ros2_message/srv/set_engine.hpp"

#include <chrono>
#include <cstdlib>
#include <memory>

using namespace std::chrono_literals;

void startEngine(rclcpp::Node::SharedPtr node, rclcpp::Client<wmx_ros2_message::srv::SetEngine>::SharedPtr setEngineClient);
void stopEngine(rclcpp::Node::SharedPtr node, rclcpp::Client<wmx_ros2_message::srv::SetEngine>::SharedPtr setEngineClient);

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  auto node = rclcpp::Node::make_shared("wmx_engine_example");

  auto setEngineClient = node->create_client<wmx_ros2_message::srv::SetEngine>("/wmx/engine/set_engine");

  startEngine(node, setEngineClient);

  std::this_thread::sleep_for(std::chrono::seconds(3)); 

  rclcpp::shutdown();
  return 0;
}

void startEngine(rclcpp::Node::SharedPtr node, rclcpp::Client<wmx_ros2_message::srv::SetEngine>::SharedPtr setEngineClient){
  if (!setEngineClient->wait_for_service(5s)) {
    RCLCPP_ERROR(node->get_logger(), "Service /wmx/engine/set_engine not available after waiting");
    rclcpp::shutdown();
    return;
  }

  auto request = std::make_shared<wmx_ros2_message::srv::SetEngine::Request>();
  request->data = true;
  request->path = "/opt/lmx";
  request->name = "wmx_ros2_general";

  auto future_result = setEngineClient->async_send_request(request);

  if (rclcpp::spin_until_future_complete(node, future_result, 5s) ==
      rclcpp::FutureReturnCode::SUCCESS)
  {
    auto response = future_result.get();
    if (response->success) {
      RCLCPP_INFO(node->get_logger(), "Success: %s", response->message.c_str());
    } else {
      RCLCPP_ERROR(node->get_logger(), "Failure: %s", response->message.c_str());
    }
  } 
  else {
    RCLCPP_ERROR(node->get_logger(), "Failed to call service /wmx/engine/set_engine");
  }
}

void stopEngine(rclcpp::Node::SharedPtr node, rclcpp::Client<wmx_ros2_message::srv::SetEngine>::SharedPtr setEngineClient){
  if (!setEngineClient->wait_for_service(5s)) {
    RCLCPP_ERROR(node->get_logger(), "Service /wmx/engine/set_engine not available after waiting");
    rclcpp::shutdown();
    return;
  }

  auto request = std::make_shared<wmx_ros2_message::srv::SetEngine::Request>();
  request->data = false;

  auto future_result = setEngineClient->async_send_request(request);

  if (rclcpp::spin_until_future_complete(node, future_result, 5s) ==
      rclcpp::FutureReturnCode::SUCCESS)
  {
    auto response = future_result.get();
    if (response->success) {
      RCLCPP_INFO(node->get_logger(), "Success: %s", response->message.c_str());
    } else {
      RCLCPP_ERROR(node->get_logger(), "Failure: %s", response->message.c_str());
    }
  } 
  else {
    RCLCPP_ERROR(node->get_logger(), "Failed to call service /wmx/engine/set_engine");
  }
}
