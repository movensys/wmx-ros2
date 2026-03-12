#ifndef WMX_IO_NODE_HPP
#define WMX_IO_NODE_HPP

#include <iostream>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"

#include "wmx_ros2_message/srv/get_io_bit.hpp"
#include "wmx_ros2_message/srv/get_io_bytes.hpp"
#include "wmx_ros2_message/srv/set_io_bit.hpp"
#include "wmx_ros2_message/srv/set_io_bytes.hpp"

#include "WMX3Api.h"
#include "IOApi.h"

using std::placeholders::_1;
using std::placeholders::_2;
using namespace wmx3Api;

class WmxIoNode : public rclcpp::Node {
public:
    WmxIoNode();
    ~WmxIoNode();

private:
    bool initialized_ = false;
    int err_;
    char errString_[256];
    char buffer_[512];

    WMX3Api wmx3Lib_;
    std::unique_ptr<Io> wmxIo_;

    rclcpp::TimerBase::SharedPtr retryTimer_;
    int deviceRetryCount_ = 0;
    static constexpr int kMaxDeviceRetries = 30;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr engineReadySub_;

    rclcpp::Service<wmx_ros2_message::srv::GetIoBit>::SharedPtr getInputBitService_;
    rclcpp::Service<wmx_ros2_message::srv::GetIoBit>::SharedPtr getOutputBitService_;
    rclcpp::Service<wmx_ros2_message::srv::GetIoBytes>::SharedPtr getInputBytesService_;
    rclcpp::Service<wmx_ros2_message::srv::GetIoBytes>::SharedPtr getOutputBytesService_;
    rclcpp::Service<wmx_ros2_message::srv::SetIoBit>::SharedPtr setOutputBitService_;
    rclcpp::Service<wmx_ros2_message::srv::SetIoBytes>::SharedPtr setOutputBytesService_;

    void onEngineReady(const std_msgs::msg::Bool::SharedPtr msg);

    void getInputBit(const std::shared_ptr<wmx_ros2_message::srv::GetIoBit::Request> request,
                     std::shared_ptr<wmx_ros2_message::srv::GetIoBit::Response> response);
    void getOutputBit(const std::shared_ptr<wmx_ros2_message::srv::GetIoBit::Request> request,
                      std::shared_ptr<wmx_ros2_message::srv::GetIoBit::Response> response);
    void getInputBytes(const std::shared_ptr<wmx_ros2_message::srv::GetIoBytes::Request> request,
                       std::shared_ptr<wmx_ros2_message::srv::GetIoBytes::Response> response);
    void getOutputBytes(const std::shared_ptr<wmx_ros2_message::srv::GetIoBytes::Request> request,
                        std::shared_ptr<wmx_ros2_message::srv::GetIoBytes::Response> response);
    void setOutputBit(const std::shared_ptr<wmx_ros2_message::srv::SetIoBit::Request> request,
                      std::shared_ptr<wmx_ros2_message::srv::SetIoBit::Response> response);
    void setOutputBytes(const std::shared_ptr<wmx_ros2_message::srv::SetIoBytes::Request> request,
                        std::shared_ptr<wmx_ros2_message::srv::SetIoBytes::Response> response);
};

#endif  // WMX_IO_NODE_HPP
