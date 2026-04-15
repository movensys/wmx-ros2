#include <memory>
#include <thread>
#include <sstream>
#include <chrono>

#include "WMX3Api.h"
#include "IOApi.h"

#include "rclcpp/rclcpp.hpp"

#include "std_srvs/srv/set_bool.hpp"
#include "std_msgs/msg/bool.hpp"

using namespace wmx3Api;

class GripperController : public rclcpp::Node {
public:
  GripperController();
  ~GripperController();

  std::vector<int64_t> gripperAddress;
  std::string wmxGripperTopic_;

  int err_;
  char errString_[256];

private:
  bool initialized_ = false;

  WMX3Api wmx3Lib_;
  Io Wmx3Lib_Io_;

  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr engineReadySub_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr setGripperService_;

  // Service callback declaration
  void setGripper(const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                        std::shared_ptr<std_srvs::srv::SetBool::Response> response);

  void setRosParameter();
  void onEngineReady(std_msgs::msg::Bool::ConstSharedPtr msg);
};

GripperController::GripperController() : Node("gripper_controller"){
  setRosParameter();

  engineReadySub_ = this->create_subscription<std_msgs::msg::Bool>(
    "/wmx/engine/ready", 1,
    std::bind(&GripperController::onEngineReady, this, std::placeholders::_1));

  RCLCPP_INFO(this->get_logger(), "gripper_controller waiting for engine...");
}

GripperController::~GripperController(){
  RCLCPP_INFO(this->get_logger(), "Stop joint_trajectory_controller");

  if (initialized_) {
    err_ = wmx3Lib_.CloseDevice();
    if (err_ != ErrorCode::None) {
      wmx3Lib_.ErrorToString(err_, errString_, sizeof(errString_));
      RCLCPP_ERROR(this->get_logger(), "Failed to close device");
    } else {
      RCLCPP_INFO(this->get_logger(), "Device closed");
    }
  }

  RCLCPP_INFO(this->get_logger(), "gripper_controller is stopped");
}

void GripperController::onEngineReady(std_msgs::msg::Bool::ConstSharedPtr msg) {
  if (!msg->data || initialized_) {
    return;
  }

  RCLCPP_INFO(this->get_logger(), "Engine ready — initializing Gripper Controller");

  unsigned int timeout = 10000;
  err_ = wmx3Lib_.CreateDevice("/opt/lmx/", DeviceType::DeviceTypeNormal, timeout);
  wmx3Lib_.SetDeviceName("gripper_controller");

  if (err_ != ErrorCode::None) {
    wmx3Lib_.ErrorToString(err_, errString_, sizeof(errString_));
    RCLCPP_ERROR(this->get_logger(),
                 "Failed to attach to device. Error=%d (%s)", err_, errString_);
    return;
  }

  RCLCPP_INFO(this->get_logger(), "Attached to WMX3 device");

  Wmx3Lib_Io_ = Io(&wmx3Lib_);
  setGripperService_ = this->create_service<std_srvs::srv::SetBool>(wmxGripperTopic_,
                    std::bind(&GripperController::setGripper, this,
                    std::placeholders::_1, std::placeholders::_2));

  initialized_ = true;
  engineReadySub_.reset();

  RCLCPP_INFO(this->get_logger(), "gripper_controller is ready");
}

void GripperController::setRosParameter(){
  this->declare_parameter<std::string>("wmx_gripper_topic", "/wmx_gripper_topic/no_param");
  this->declare_parameter<std::vector<int64_t>>("gripper_address", std::vector<int64_t>{0, 0});

  this->get_parameter("wmx_gripper_topic", wmxGripperTopic_);
  this->get_parameter("gripper_address", gripperAddress);

  // Print parameter values
  RCLCPP_INFO(this->get_logger(), "===== ROS2 Parameters =====");
  RCLCPP_INFO(this->get_logger(), "wmx_gripper_topic: %s", wmxGripperTopic_.c_str());
  RCLCPP_INFO(this->get_logger(), "gripper_address: [%ld, %ld]", gripperAddress[0], gripperAddress[1]);
  RCLCPP_INFO(this->get_logger(), "===========================");
}

void GripperController::setGripper(const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                                        std::shared_ptr<std_srvs::srv::SetBool::Response> response){
  if (request->data) {
    err_ = Wmx3Lib_Io_.SetOutBit(gripperAddress[0], gripperAddress[1], 1);
    if (err_ != ErrorCode::None) {
      wmx3Lib_.ErrorToString(err_, errString_, sizeof(errString_));
      RCLCPP_ERROR(this->get_logger(), "Gripper fails to Close: %s", errString_);
      response->success = false;
      response->message = "Failed to close gripper";
    }
    else {
      RCLCPP_INFO(this->get_logger(), "Gripper success to Close");
      response->success = true;
      response->message = "Gripper closed successfully";
    }
  }
  else {
    err_ = Wmx3Lib_Io_.SetOutBit(gripperAddress[0], gripperAddress[1], 0);
    if (err_ != ErrorCode::None) {
      wmx3Lib_.ErrorToString(err_, errString_, sizeof(errString_));
      RCLCPP_ERROR(this->get_logger(), "Gripper fails to Open: %s", errString_);
      response->success = false;
      response->message = "Failed to open gripper";
    }
    else {
      RCLCPP_INFO(this->get_logger(), "Gripper success to Open");
      response->success = true;
      response->message = "Gripper opened successfully";
    }
  }
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<GripperController>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}