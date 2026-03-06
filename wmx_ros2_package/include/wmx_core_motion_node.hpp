#ifndef WMX_CORE_MOTION_NODE_HPP
#define WMX_CORE_MOTION_NODE_HPP

#include <iostream>
#include <memory>
#include <string>
#include <vector>
#include <sstream>
#include <chrono>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_srvs/srv/set_bool.hpp"

#include "wmx_ros2_message/srv/set_axis.hpp"
#include "wmx_ros2_message/srv/set_axis_gear_ratio.hpp"
#include "wmx_ros2_message/srv/load_wmx_params.hpp"
#include "wmx_ros2_message/srv/get_wmx_params.hpp"
#include "wmx_ros2_message/msg/axis_velocity.hpp"
#include "wmx_ros2_message/msg/axis_state.hpp"
#include "wmx_ros2_message/msg/axis_pose.hpp"

#include "WMX3Api.h"
#include "CoreMotionApi.h"

using std::placeholders::_1;
using std::placeholders::_2;
using namespace wmx3Api;

class WmxCoreMotionNode : public rclcpp::Node {
public:
    WmxCoreMotionNode();
    ~WmxCoreMotionNode();

private:
    bool initialized_ = false;
    int axisCount_ = 2;
    int err_;
    char errString_[256];
    char buffer_[512];
    const int rate_ = 100;

    WMX3Api wmx3Lib_;
    std::unique_ptr<CoreMotion> wmx3LibCm_;
    CoreMotionStatus cmStatus_;

    wmx3Api::Velocity::VelCommand velocity_;
    wmx3Api::Motion::PosCommand position_;
    Config::HomeParam homeParam_;

    rclcpp::TimerBase::SharedPtr axisStateTimer_;
    wmx_ros2_message::msg::AxisState axisStateMsg_;

    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr engineReadySub_;
    rclcpp::Publisher<wmx_ros2_message::msg::AxisState>::SharedPtr axisStatePub_;
    rclcpp::Subscription<wmx_ros2_message::msg::AxisVelocity>::SharedPtr axisVelSub_;
    rclcpp::Subscription<wmx_ros2_message::msg::AxisPose>::SharedPtr axisPoseSub_;
    rclcpp::Subscription<wmx_ros2_message::msg::AxisPose>::SharedPtr axisPoseRelativeSub_;

    rclcpp::Service<wmx_ros2_message::srv::SetAxis>::SharedPtr setAxisOnService_;
    rclcpp::Service<wmx_ros2_message::srv::SetAxis>::SharedPtr clearAlarmService_;
    rclcpp::Service<wmx_ros2_message::srv::SetAxis>::SharedPtr setAxisModeService_;
    rclcpp::Service<wmx_ros2_message::srv::SetAxis>::SharedPtr setAxisPolarityService_;
    rclcpp::Service<wmx_ros2_message::srv::SetAxisGearRatio>::SharedPtr setAxisGearRatioService_;
    rclcpp::Service<wmx_ros2_message::srv::SetAxis>::SharedPtr setHomingService_;
    rclcpp::Service<wmx_ros2_message::srv::LoadWmxParams>::SharedPtr loadParamsService_;
    rclcpp::Service<wmx_ros2_message::srv::GetWmxParams>::SharedPtr  getParamsService_;

    void onEngineReady(const std_msgs::msg::Bool::SharedPtr msg);
    void axisStateStep();

    void axisPoseCallback(const wmx_ros2_message::msg::AxisPose::SharedPtr msg);
    void axisPoseRelativeCallback(const wmx_ros2_message::msg::AxisPose::SharedPtr msg);
    void axisVelCallback(const wmx_ros2_message::msg::AxisVelocity::SharedPtr msg);

    void setAxisOn(const std::shared_ptr<wmx_ros2_message::srv::SetAxis::Request> request,
                   std::shared_ptr<wmx_ros2_message::srv::SetAxis::Response> response);
    void setAxisMode(const std::shared_ptr<wmx_ros2_message::srv::SetAxis::Request> request,
                     std::shared_ptr<wmx_ros2_message::srv::SetAxis::Response> response);
    void clearAlarm(const std::shared_ptr<wmx_ros2_message::srv::SetAxis::Request> request,
                    std::shared_ptr<wmx_ros2_message::srv::SetAxis::Response> response);
    void setAxisPolarity(const std::shared_ptr<wmx_ros2_message::srv::SetAxis::Request> request,
                         std::shared_ptr<wmx_ros2_message::srv::SetAxis::Response> response);
    void setAxisGearRatio(const std::shared_ptr<wmx_ros2_message::srv::SetAxisGearRatio::Request> request,
                          std::shared_ptr<wmx_ros2_message::srv::SetAxisGearRatio::Response> response);
    void setHoming(const std::shared_ptr<wmx_ros2_message::srv::SetAxis::Request> request,
                   std::shared_ptr<wmx_ros2_message::srv::SetAxis::Response> response);
    void loadWmxParams(const std::shared_ptr<wmx_ros2_message::srv::LoadWmxParams::Request> request,
                       std::shared_ptr<wmx_ros2_message::srv::LoadWmxParams::Response> response);
    void getWmxParams(const std::shared_ptr<wmx_ros2_message::srv::GetWmxParams::Request> request,
                      std::shared_ptr<wmx_ros2_message::srv::GetWmxParams::Response> response);
};

#endif  // WMX_CORE_MOTION_NODE_HPP
