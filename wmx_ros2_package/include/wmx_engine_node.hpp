#ifndef WMX_ENGINE_NODE_HPP
#define WMX_ENGINE_NODE_HPP

#include <iostream>
#include <memory>
#include <string>
#include <chrono>
#include <thread>
#include <atomic>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "std_srvs/srv/trigger.hpp"

#include "wmx_ros2_message/srv/set_engine.hpp"

#include "WMX3Api.h"
#include "EcApi.h"

class WmxEngineNode : public rclcpp::Node
{
public:
  WmxEngineNode();
  ~WmxEngineNode();

private:
  wmx3Api::WMX3Api wmx3Lib_;
  wmx3Api::ecApi::Ecat wmx3Lib_Ecat_;
  std::atomic<bool> commStarted_{false};
  std::atomic<bool> startComplete_{false};
  std::thread startThread_;

  rclcpp::TimerBase::SharedPtr readyTimer_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr engineReadyPub_;

  rclcpp::Service<wmx_ros2_message::srv::SetEngine>::SharedPtr setEngineService_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr setCommService_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr getEngineStatusService_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr scanNetworkService_;

  void startEngine();
  void stopEngine();
  void stopCommunication();
  void publishReady();

  void setEngine(
    const std::shared_ptr<wmx_ros2_message::srv::SetEngine::Request> request,
    std::shared_ptr<wmx_ros2_message::srv::SetEngine::Response> response);
  void setComm(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    std::shared_ptr<std_srvs::srv::SetBool::Response> response);
  void getEngineStatus(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);
  void scanNetwork(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);
};

#endif  // WMX_ENGINE_NODE_HPP
