#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "wmx_ros2_message/srv/set_engine.hpp"
#include "wmx_ros2_message/srv/set_axis.hpp"
#include "wmx_ros2_message/srv/set_axis_gear_ratio.hpp"

#include "wmx_ros2_message/msg/axis_velocity.hpp"
#include "wmx_ros2_message/msg/axis_pose.hpp"

#include <chrono>
#include <memory>
#include <string>
#include <iostream>

using namespace std::chrono_literals;
using namespace std;

wmx_ros2_message::msg::AxisVelocity axisVelMsg_;
wmx_ros2_message::msg::AxisPose axisPoseMsg_;

void setEngine(const std::shared_ptr<rclcpp::Node>& node, rclcpp::Client<wmx_ros2_message::srv::SetEngine>::SharedPtr client,
        bool data, const std::string & path, const std::string & name);

void setComm(const std::shared_ptr<rclcpp::Node>& node, rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr client,
        bool data);

void getEngineStatus(const std::shared_ptr<rclcpp::Node>& node, rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr client);

void setAxisGearRatio(const std::shared_ptr<rclcpp::Node>& node, rclcpp::Client<wmx_ros2_message::srv::SetAxisGearRatio>::SharedPtr client,
        const std::vector<int>& index, const std::vector<double>& numerator, const std::vector<double>& denumerator);

void setAxisPolarity(const std::shared_ptr<rclcpp::Node>& node, rclcpp::Client<wmx_ros2_message::srv::SetAxis>::SharedPtr client,
        const std::vector<int>& index, const std::vector<int>& data);

void setAxisOn(const std::shared_ptr<rclcpp::Node>& node, rclcpp::Client<wmx_ros2_message::srv::SetAxis>::SharedPtr client,
        const std::vector<int>& index, const std::vector<int>& data);

void setAxisMode(const std::shared_ptr<rclcpp::Node>& node, rclcpp::Client<wmx_ros2_message::srv::SetAxis>::SharedPtr client,
        const std::vector<int>& index, const std::vector<int>& data);

void setClearAlarm(const std::shared_ptr<rclcpp::Node>& node, rclcpp::Client<wmx_ros2_message::srv::SetAxis>::SharedPtr client,
        const std::vector<int>& index);

void setHoming(const std::shared_ptr<rclcpp::Node>& node, rclcpp::Client<wmx_ros2_message::srv::SetAxis>::SharedPtr client,
        const std::vector<int>& index);

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("wmx_ros2_general_test");

  rclcpp::Publisher<wmx_ros2_message::msg::AxisVelocity>::SharedPtr axisVelPub_ = 
                    node->create_publisher<wmx_ros2_message::msg::AxisVelocity>("/wmx/axis/velocity", 1); 

  rclcpp::Publisher<wmx_ros2_message::msg::AxisPose>::SharedPtr axisPosePub_ = 
                    node->create_publisher<wmx_ros2_message::msg::AxisPose>("/wmx/axis/position", 1); 
  
  rclcpp::Publisher<wmx_ros2_message::msg::AxisPose>::SharedPtr axisPoseRelativePub_ = 
                    node->create_publisher<wmx_ros2_message::msg::AxisPose>("/wmx/axis/position/relative", 1); 
  
  
  auto setEngineClient = node->create_client<wmx_ros2_message::srv::SetEngine>("/wmx/engine/set_device");
  auto setCommClient = node->create_client<std_srvs::srv::SetBool>("/wmx/engine/set_comm");
  auto getEngineStatusClient = node->create_client<std_srvs::srv::Trigger>("/wmx/engine/get_status");
  
  auto setAxisOnClient_ = node->create_client<wmx_ros2_message::srv::SetAxis>("/wmx/axis/set_on");
  auto setAxisGearRatioClient_ = node->create_client<wmx_ros2_message::srv::SetAxisGearRatio>("/wmx/axis/set_gear_ratio");
  auto setAxisModeClient_ = node->create_client<wmx_ros2_message::srv::SetAxis>("/wmx/axis/set_mode");
  auto setAxisPolarityClient_ = node->create_client<wmx_ros2_message::srv::SetAxis>("/wmx/axis/set_polarity");
  auto clearAlarmClient_ = node->create_client<wmx_ros2_message::srv::SetAxis>("/wmx/axis/clear_alarm");
  auto setHomingClient_ = node->create_client<wmx_ros2_message::srv::SetAxis>("/wmx/axis/homing");

  setEngine(node, setEngineClient, true, "/opt/lmx/", "wmx_ros2_general_test");  // Start engine
  rclcpp::sleep_for(std::chrono::seconds(1));

  getEngineStatus(node, getEngineStatusClient); //get engine status
  rclcpp::sleep_for(std::chrono::seconds(1));

  setComm(node, setCommClient, true);  // Start comm
  rclcpp::sleep_for(std::chrono::seconds(1));

  getEngineStatus(node, getEngineStatusClient); //get engine status
  rclcpp::sleep_for(std::chrono::seconds(1));

  setAxisGearRatio(node, setAxisGearRatioClient_, {0, 1}, {8388608.0f, 8388608.0f}, {6.28319f, 6.28319f}); //set gear ratio
  rclcpp::sleep_for(std::chrono::seconds(1));

  setAxisPolarity(node, setAxisPolarityClient_, {0, 1}, {1, -1}); //set axis polarity
  rclcpp::sleep_for(std::chrono::seconds(1));

  setClearAlarm(node, clearAlarmClient_, {0, 1}); //clear alarm
  rclcpp::sleep_for(std::chrono::seconds(1));
  
  setAxisMode(node, setAxisModeClient_, {0, 1}, {1, 1}); //set axis mode
  rclcpp::sleep_for(std::chrono::seconds(1));

  setAxisOn(node, setAxisOnClient_, {0, 1}, {1, 1}); //set servo on
  rclcpp::sleep_for(std::chrono::seconds(1));

  setHoming(node, setHomingClient_, {0, 1}); //set homing
  rclcpp::sleep_for(std::chrono::seconds(1));

  RCLCPP_INFO(node->get_logger(), "Publish /wmx/axis/velocity 1 rad/s velocity");
  axisVelMsg_.index = {0, 1};
  axisVelMsg_.profile = "Trapezoidal";
  axisVelMsg_.velocity = {1, 1};
  axisVelMsg_.acc = {0.5, 0.5};
  axisVelMsg_.dec = {0.5, 0.5};
  axisVelPub_->publish(axisVelMsg_);
  rclcpp::sleep_for(std::chrono::seconds(10));

  RCLCPP_INFO(node->get_logger(), "Publish /wmx/axis/velocity 0 rad/s velocity");
  axisVelMsg_.index = {0, 1};
  axisVelMsg_.profile = {"Trapezoidal", "Trapezoidal"};
  axisVelMsg_.velocity = {0, 0};
  axisVelMsg_.acc = {0.5, 0.5};
  axisVelMsg_.dec = {0.5, 0.5};
  axisVelPub_->publish(axisVelMsg_);
  rclcpp::sleep_for(std::chrono::seconds(5));

  setAxisOn(node, setAxisOnClient_, {0, 1}, {0, 0}); //set servo off
  rclcpp::sleep_for(std::chrono::seconds(1));

  setAxisMode(node, setAxisModeClient_, {0, 1}, {0, 0}); //set axis mode
  rclcpp::sleep_for(std::chrono::seconds(1));

  setAxisOn(node, setAxisOnClient_, {0, 1}, {1, 1}); //set servo on
  rclcpp::sleep_for(std::chrono::seconds(1));

  setHoming(node, setHomingClient_, {0, 1}); //set homing
  rclcpp::sleep_for(std::chrono::seconds(1));
  
  RCLCPP_INFO(node->get_logger(), "Publish /wmx/axis/pose 5 rad");
  axisPoseMsg_.index = {0, 1};
  axisPoseMsg_.target = {5.0, 5.0};
  axisPoseMsg_.profile = "Trapezoidal";
  axisPoseMsg_.velocity = {1, 1};
  axisPoseMsg_.acc = {0.5, 0.5};
  axisPoseMsg_.dec = {0.5, 0.5};
  axisPosePub_->publish(axisPoseMsg_);
  rclcpp::sleep_for(std::chrono::seconds(10));

  RCLCPP_INFO(node->get_logger(), "Publish /wmx/axis/pose -2 rad");
  axisPoseMsg_.index = {0, 1};
  axisPoseMsg_.target = {-2.0, -2.0};
  axisPoseMsg_.profile = "Trapezoidal";
  axisPoseMsg_.velocity = {1, 1};
  axisPoseMsg_.acc = {0.5, 0.5};
  axisPoseMsg_.dec = {0.5, 0.5};
  axisPosePub_->publish(axisPoseMsg_);
  rclcpp::sleep_for(std::chrono::seconds(10));

  setHoming(node, setHomingClient_, {0, 1}); //set homing
  rclcpp::sleep_for(std::chrono::seconds(1));
  
  RCLCPP_INFO(node->get_logger(), "Publish /wmx/axis/pose/relative 5 rad");
  axisPoseMsg_.index = {0, 1};
  axisPoseMsg_.target = {5.0, 5.0};
  axisPoseMsg_.profile = "Trapezoidal";
  axisPoseMsg_.velocity = {1, 1};
  axisPoseMsg_.acc = {0.5, 0.5};
  axisPoseMsg_.dec = {0.5, 0.5};
  axisPoseRelativePub_->publish(axisPoseMsg_);
  rclcpp::sleep_for(std::chrono::seconds(10));

  RCLCPP_INFO(node->get_logger(), "Publish /wmx/axis/relative -2 rad");
  axisPoseMsg_.index = {0, 1};
  axisPoseMsg_.target = {-2.0, -2.0};
  axisPoseMsg_.profile = "Trapezoidal";
  axisPoseMsg_.velocity = {1, 1};
  axisPoseMsg_.acc = {0.5, 0.5};
  axisPoseMsg_.dec = {0.5, 0.5};
  axisPoseRelativePub_->publish(axisPoseMsg_);
  rclcpp::sleep_for(std::chrono::seconds(10));

  setAxisOn(node, setAxisOnClient_, {0, 1}, {0, 0}); //set servo off
  rclcpp::sleep_for(std::chrono::seconds(1));
  
  setComm(node, setCommClient, false);  // Stop comm
  rclcpp::sleep_for(std::chrono::seconds(1));
  
  setEngine(node, setEngineClient, false, "/opt/lmx/", "wmx_ros2_general_test"); // Stop engine
  rclcpp::sleep_for(std::chrono::seconds(1));

  rclcpp::shutdown();
  return 0;
}

void setHoming(const std::shared_ptr<rclcpp::Node>& node, rclcpp::Client<wmx_ros2_message::srv::SetAxis>::SharedPtr client,
  const std::vector<int>& index){

  RCLCPP_INFO(node->get_logger(), "Calling /wmx/axis/homing");

  auto request = std::make_shared<wmx_ros2_message::srv::SetAxis::Request>();
  request->index = index;
          
  while (!client->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(node->get_logger(), "Interrupted while waiting for the service. Exiting.");
      return;
    }
    RCLCPP_INFO(node->get_logger(), "Service not available, waiting again...");
  }
          
  auto result = client->async_send_request(request);
          
  if (rclcpp::spin_until_future_complete(node, result) == rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_INFO(node->get_logger(), "Status: %s", result.get()->success ? "true" : "false");
    RCLCPP_INFO(node->get_logger(), "Message: %s", result.get()->message.c_str());
  } 
  else {
    RCLCPP_ERROR(node->get_logger(), "Failed to call service");
  }
  cout<<endl<<endl;
}

void setClearAlarm(const std::shared_ptr<rclcpp::Node>& node, rclcpp::Client<wmx_ros2_message::srv::SetAxis>::SharedPtr client,
  const std::vector<int>& index){

  RCLCPP_INFO(node->get_logger(), "Calling /wmx/axis/clear_alarm");

  auto request = std::make_shared<wmx_ros2_message::srv::SetAxis::Request>();
  request->index = index;
          
  while (!client->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(node->get_logger(), "Interrupted while waiting for the service. Exiting.");
      return;
    }
    RCLCPP_INFO(node->get_logger(), "Service not available, waiting again...");
  }
          
  auto result = client->async_send_request(request);
          
  if (rclcpp::spin_until_future_complete(node, result) == rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_INFO(node->get_logger(), "Status: %s", result.get()->success ? "true" : "false");
    RCLCPP_INFO(node->get_logger(), "Message: %s", result.get()->message.c_str());
  } 
  else {
    RCLCPP_ERROR(node->get_logger(), "Failed to call service");
  }
  cout<<endl<<endl;
}

void setAxisMode(const std::shared_ptr<rclcpp::Node>& node, rclcpp::Client<wmx_ros2_message::srv::SetAxis>::SharedPtr client,
          const std::vector<int>& index, const std::vector<int>& data){
  RCLCPP_INFO(node->get_logger(), "Calling /wmx/axis/set_mode");

  auto request = std::make_shared<wmx_ros2_message::srv::SetAxis::Request>();
  request->index = index;
  request->data = data;
          
  while (!client->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(node->get_logger(), "Interrupted while waiting for the service. Exiting.");
      return;
    }
    RCLCPP_INFO(node->get_logger(), "Service not available, waiting again...");
  }
          
  auto result = client->async_send_request(request);
          
  if (rclcpp::spin_until_future_complete(node, result) == rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_INFO(node->get_logger(), "Status: %s", result.get()->success ? "true" : "false");
    RCLCPP_INFO(node->get_logger(), "Message: %s", result.get()->message.c_str());
  } 
  else {
    RCLCPP_ERROR(node->get_logger(), "Failed to call service");
  }
  cout<<endl<<endl;
}

void setAxisOn(const std::shared_ptr<rclcpp::Node>& node, rclcpp::Client<wmx_ros2_message::srv::SetAxis>::SharedPtr client,
          const std::vector<int>& index, const std::vector<int>& data){

  RCLCPP_INFO(node->get_logger(), "Calling /wmx/axis/set_on");

  auto request = std::make_shared<wmx_ros2_message::srv::SetAxis::Request>();
  request->index = index;
  request->data = data;
          
  while (!client->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(node->get_logger(), "Interrupted while waiting for the service. Exiting.");
      return;
    }
    RCLCPP_INFO(node->get_logger(), "Service not available, waiting again...");
  }
          
  auto result = client->async_send_request(request);
          
  if (rclcpp::spin_until_future_complete(node, result) == rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_INFO(node->get_logger(), "Status: %s", result.get()->success ? "true" : "false");
    RCLCPP_INFO(node->get_logger(), "Message: %s", result.get()->message.c_str());
  } 
  else {
    RCLCPP_ERROR(node->get_logger(), "Failed to call service");
  }
  cout<<endl<<endl;
}


void setAxisPolarity(const std::shared_ptr<rclcpp::Node>& node, rclcpp::Client<wmx_ros2_message::srv::SetAxis>::SharedPtr client,
              const std::vector<int>& index, const std::vector<int>& data){
  RCLCPP_INFO(node->get_logger(), "Calling /wmx/axis/set_polarity");

  auto request = std::make_shared<wmx_ros2_message::srv::SetAxis::Request>();
  request->index = index;
  request->data = data;
          
  while (!client->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(node->get_logger(), "Interrupted while waiting for the service. Exiting.");
      return;
    }
    RCLCPP_INFO(node->get_logger(), "Service not available, waiting again...");
  }
          
  auto result = client->async_send_request(request);
          
  if (rclcpp::spin_until_future_complete(node, result) == rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_INFO(node->get_logger(), "Status: %s", result.get()->success ? "true" : "false");
    RCLCPP_INFO(node->get_logger(), "Message: %s", result.get()->message.c_str());
  } 
  else {
    RCLCPP_ERROR(node->get_logger(), "Failed to call service");
  }
  std::cout << std::endl << std::endl;
}

void setAxisGearRatio(const std::shared_ptr<rclcpp::Node>& node, rclcpp::Client<wmx_ros2_message::srv::SetAxisGearRatio>::SharedPtr client,
            const std::vector<int>& index, const std::vector<double>& numerator, const std::vector<double>& denumerator){
  RCLCPP_INFO(node->get_logger(), "Calling /wmx/axis/set_gear_ratio");

  auto request = std::make_shared<wmx_ros2_message::srv::SetAxisGearRatio::Request>();
  request->index = index;
  request->numerator = numerator;
  request->denumerator = denumerator;

  while (!client->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(node->get_logger(), "Interrupted while waiting for the service. Exiting.");
      return;
    }
    RCLCPP_INFO(node->get_logger(), "Service not available, waiting again...");
  }

  auto result = client->async_send_request(request);

  if (rclcpp::spin_until_future_complete(node, result) == rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_INFO(node->get_logger(), "Status: %s", result.get()->success ? "true" : "false");
    RCLCPP_INFO(node->get_logger(), "Message: %s", result.get()->message.c_str());
  } 
  else {
    RCLCPP_ERROR(node->get_logger(), "Failed to call service");
  }

  std::cout << std::endl << std::endl;
}


void getEngineStatus(const std::shared_ptr<rclcpp::Node>& node, rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr client) {
  RCLCPP_INFO(node->get_logger(), "Calling /wmx/engine/get_status");

  auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
  
  while (!client->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(node->get_logger(), "Interrupted while waiting for the service. Exiting.");
    return;
    }
    RCLCPP_INFO(node->get_logger(), "Service not available, waiting again...");
  }

  auto result = client->async_send_request(request);

  if (rclcpp::spin_until_future_complete(node, result) == rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_INFO(node->get_logger(), "Status: %s", result.get()->success ? "true" : "false");
    RCLCPP_INFO(node->get_logger(), "Message: %s", result.get()->message.c_str());
  }  
  else {
    RCLCPP_ERROR(node->get_logger(), "Failed to call service");
  }
  cout<<endl<<endl;
}


void setComm(const std::shared_ptr<rclcpp::Node>& node, rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr client,
        bool data) {
  RCLCPP_INFO(node->get_logger(), "Calling /wmx/engine/set_comm");

  auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
  request->data = data;

  while (!client->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(node->get_logger(), "Interrupted while waiting for the service. Exiting.");
      return;
    }
    RCLCPP_INFO(node->get_logger(), "Service not available, waiting again...");
  }

  auto result = client->async_send_request(request);

  if (rclcpp::spin_until_future_complete(node, result) == rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_INFO(node->get_logger(), "Status: %s", result.get()->success ? "true" : "false");
    RCLCPP_INFO(node->get_logger(), "Message: %s", result.get()->message.c_str());
  } 
  else {
    RCLCPP_ERROR(node->get_logger(), "Failed to call service");
  }
  cout<<endl<<endl;
}

void setEngine(const std::shared_ptr<rclcpp::Node>& node, rclcpp::Client<wmx_ros2_message::srv::SetEngine>::SharedPtr client,
        bool data, const std::string & path, const std::string & name) {
  RCLCPP_INFO(node->get_logger(), "Calling /wmx/engine/set_device");

  auto request = std::make_shared<wmx_ros2_message::srv::SetEngine::Request>();
  request->data = data;
  request->path = path;
  request->name = name;

  while (!client->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(node->get_logger(), "Interrupted while waiting for the service. Exiting.");
      return;
    }
    RCLCPP_INFO(node->get_logger(), "Service not available, waiting again...");
  }

  auto result = client->async_send_request(request);

  if (rclcpp::spin_until_future_complete(node, result) == rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_INFO(node->get_logger(), "Status: %s", result.get()->success ? "true" : "false");
    RCLCPP_INFO(node->get_logger(), "Message: %s", result.get()->message.c_str());
  } 
  else {
    RCLCPP_ERROR(node->get_logger(), "Failed to call service");
  }
  cout<<endl<<endl;
}