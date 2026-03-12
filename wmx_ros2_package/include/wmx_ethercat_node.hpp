#ifndef WMX_ETHERCAT_NODE_HPP
#define WMX_ETHERCAT_NODE_HPP

#include <iostream>
#include <memory>
#include <string>
#include <vector>
#include <chrono>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"

#include "wmx_ros2_message/srv/ecat_get_network_state.hpp"
#include "wmx_ros2_message/srv/ecat_register_read.hpp"
#include "wmx_ros2_message/srv/ecat_reset_statistics.hpp"
#include "wmx_ros2_message/srv/ecat_start_hotconnect.hpp"

#include "WMX3Api.h"
#include "EcApi.h"

using std::placeholders::_1;
using std::placeholders::_2;
using namespace wmx3Api;

class WmxEtherCatNode : public rclcpp::Node {
public:
    WmxEtherCatNode();
    ~WmxEtherCatNode();

private:
    bool initialized_ = false;
    int err_;
    char errString_[256];
    char buffer_[512];

    WMX3Api wmx3Lib_;
    wmx3Api::ecApi::Ecat wmxEcat_;

    rclcpp::TimerBase::SharedPtr retryTimer_;
    int deviceRetryCount_ = 0;
    static constexpr int kMaxDeviceRetries = 30;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr engineReadySub_;

    rclcpp::Service<wmx_ros2_message::srv::EcatGetNetworkState>::SharedPtr getNetworkStateService_;
    rclcpp::Service<wmx_ros2_message::srv::EcatRegisterRead>::SharedPtr registerReadService_;
    rclcpp::Service<wmx_ros2_message::srv::EcatResetStatistics>::SharedPtr resetStatisticsService_;
    rclcpp::Service<wmx_ros2_message::srv::EcatStartHotconnect>::SharedPtr startHotconnectService_;

    void onEngineReady(const std_msgs::msg::Bool::SharedPtr msg);

    void getNetworkState(
        const std::shared_ptr<wmx_ros2_message::srv::EcatGetNetworkState::Request> request,
        std::shared_ptr<wmx_ros2_message::srv::EcatGetNetworkState::Response> response);

    void registerRead(
        const std::shared_ptr<wmx_ros2_message::srv::EcatRegisterRead::Request> request,
        std::shared_ptr<wmx_ros2_message::srv::EcatRegisterRead::Response> response);

    void resetStatistics(
        const std::shared_ptr<wmx_ros2_message::srv::EcatResetStatistics::Request> request,
        std::shared_ptr<wmx_ros2_message::srv::EcatResetStatistics::Response> response);

    void startHotconnect(
        const std::shared_ptr<wmx_ros2_message::srv::EcatStartHotconnect::Request> request,
        std::shared_ptr<wmx_ros2_message::srv::EcatStartHotconnect::Response> response);
};

#endif  // WMX_ETHERCAT_NODE_HPP
