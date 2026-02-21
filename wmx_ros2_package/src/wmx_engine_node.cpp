#include "wmx_engine_node.hpp"

WmxEngineNode::WmxEngineNode() : Node("wmx_engine_node") {
    engineReadyPub_ = this->create_publisher<std_msgs::msg::Bool>("/wmx/engine/ready", 1);

    setEngineService_ = this->create_service<wmx_ros2_message::srv::SetEngine>(
        "/wmx/engine/set_device",
        std::bind(&WmxEngineNode::setEngine, this, _1, _2));

    setCommService_ = this->create_service<std_srvs::srv::SetBool>(
        "/wmx/engine/set_comm",
        std::bind(&WmxEngineNode::setComm, this, _1, _2));

    getEngineStatusService_ = this->create_service<std_srvs::srv::Trigger>(
        "/wmx/engine/get_status",
        std::bind(&WmxEngineNode::getEngineStatus, this, _1, _2));

    readyTimer_ = this->create_wall_timer(
        std::chrono::milliseconds(1000),
        std::bind(&WmxEngineNode::publishReady, this));

    startEngine();

    RCLCPP_INFO(this->get_logger(), "wmx_engine_node is ready");
}

WmxEngineNode::~WmxEngineNode() {
    stopCommunication();
    stopEngine();
    std::this_thread::sleep_for(std::chrono::seconds(3));
    RCLCPP_INFO(this->get_logger(), "wmx_engine_node stopped");
}

void WmxEngineNode::publishReady() {
    auto msg = std_msgs::msg::Bool();
    msg.data = commStarted_;
    engineReadyPub_->publish(msg);
}

void WmxEngineNode::startEngine() {
    RCLCPP_INFO(this->get_logger(), "Starting engine...");
    unsigned int timeout = 10000;
    int maxRetries = 5;
    int retryDelay = 2000;
    const int CreateDeviceLockError = 297;

    std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    for (int attempt = 0; attempt < maxRetries; attempt++) {
        if (attempt > 0) {
            RCLCPP_INFO(this->get_logger(), "Retrying device creation (attempt %d/%d)...",
                        attempt + 1, maxRetries);
            std::this_thread::sleep_for(std::chrono::milliseconds(retryDelay));
        }

        err_ = wmx3Lib_.CreateDevice("/opt/lmx/", DeviceType::DeviceTypeNormal, timeout);
        wmx3Lib_.SetDeviceName("wmx_engine_node");

        if (err_ == ErrorCode::None) {
            RCLCPP_INFO(this->get_logger(), "Device created (attempt %d)", attempt + 1);

            err_ = wmx3Lib_.StartCommunication(timeout);
            if (err_ == ErrorCode::None) {
                RCLCPP_INFO(this->get_logger(), "Communication started");
                commStarted_ = true;
            } else {
                wmx3Lib_.ErrorToString(err_, errString_, sizeof(errString_));
                RCLCPP_ERROR(this->get_logger(),
                             "Failed to start communication. Error=%d (%s)", err_, errString_);
            }
            return;
        } else {
            wmx3Lib_.ErrorToString(err_, errString_, sizeof(errString_));
            if (err_ == CreateDeviceLockError) {
                RCLCPP_WARN(this->get_logger(),
                            "Device lock error (attempt %d/%d). Waiting...",
                            attempt + 1, maxRetries);
            } else {
                RCLCPP_WARN(this->get_logger(),
                            "Failed to create device (attempt %d/%d). Error=%d (%s)",
                            attempt + 1, maxRetries, err_, errString_);
            }
        }
    }

    wmx3Lib_.ErrorToString(err_, errString_, sizeof(errString_));
    RCLCPP_ERROR(this->get_logger(),
                 "Failed to create device after %d attempts. Error=%d (%s)",
                 maxRetries, err_, errString_);
}

void WmxEngineNode::stopCommunication() {
    unsigned int timeout = 10000;
    err_ = wmx3Lib_.StopCommunication(timeout);
    if (err_ != ErrorCode::None) {
        wmx3Lib_.ErrorToString(err_, errString_, sizeof(errString_));
        RCLCPP_ERROR(this->get_logger(), "Failed to stop communication");
    } else {
        RCLCPP_INFO(this->get_logger(), "Communication stopped");
    }
    commStarted_ = false;
}

void WmxEngineNode::stopEngine() {
    err_ = wmx3Lib_.CloseDevice();
    if (err_ != ErrorCode::None) {
        wmx3Lib_.ErrorToString(err_, errString_, sizeof(errString_));
        RCLCPP_ERROR(this->get_logger(), "Failed to close device");
    } else {
        RCLCPP_INFO(this->get_logger(), "Device closed");
    }

    unsigned int timeout = 10000;
    err_ = wmx3Lib_.StopEngine(timeout);
    if (err_ != ErrorCode::None) {
        wmx3Lib_.ErrorToString(err_, errString_, sizeof(errString_));
        RCLCPP_ERROR(this->get_logger(), "Failed to stop engine");
    } else {
        RCLCPP_INFO(this->get_logger(), "Engine stopped");
    }
}

void WmxEngineNode::getEngineStatus(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response) {

    wmx3Api::EngineStatus status;
    wmx3Lib_.GetEngineStatus(&status);

    std::string status_str;
    switch (status.state) {
        case wmx3Api::EngineState::Idle:          status_str = "Idle"; break;
        case wmx3Api::EngineState::Running:       status_str = "Running"; break;
        case wmx3Api::EngineState::Communicating: status_str = "Communicating"; break;
        case wmx3Api::EngineState::Shutdown:      status_str = "Shutdown"; break;
        case wmx3Api::EngineState::Unknown:       status_str = "Unknown"; break;
        default:                                  status_str = "Invalid"; break;
    }

    response->success = true;
    response->message = status_str;
}

void WmxEngineNode::setComm(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    std::shared_ptr<std_srvs::srv::SetBool::Response> response) {

    unsigned int timeout = 10000;
    if (request->data) {
        err_ = wmx3Lib_.StartCommunication(timeout);
        if (err_ != ErrorCode::None) {
            wmx3Lib_.ErrorToString(err_, errString_, sizeof(errString_));
            snprintf(buffer_, sizeof(buffer_),
                     "Failed to start communication. Error=%d (%s)", err_, errString_);
            RCLCPP_ERROR(this->get_logger(), "%s", buffer_);
            response->success = false;
            response->message = std::string(buffer_);
        } else {
            commStarted_ = true;
            snprintf(buffer_, sizeof(buffer_), "Communication started");
            RCLCPP_INFO(this->get_logger(), "%s", buffer_);
            response->success = true;
            response->message = std::string(buffer_);
        }
    } else {
        err_ = wmx3Lib_.StopCommunication(timeout);
        if (err_ != ErrorCode::None) {
            wmx3Lib_.ErrorToString(err_, errString_, sizeof(errString_));
            snprintf(buffer_, sizeof(buffer_),
                     "Failed to stop communication. Error=%d (%s)", err_, errString_);
            RCLCPP_ERROR(this->get_logger(), "%s", buffer_);
            response->success = false;
            response->message = std::string(buffer_);
        } else {
            commStarted_ = false;
            snprintf(buffer_, sizeof(buffer_), "Communication stopped");
            RCLCPP_INFO(this->get_logger(), "%s", buffer_);
            response->success = true;
            response->message = std::string(buffer_);
        }
    }
}

void WmxEngineNode::setEngine(
    const std::shared_ptr<wmx_ros2_message::srv::SetEngine::Request> request,
    std::shared_ptr<wmx_ros2_message::srv::SetEngine::Response> response) {

    unsigned int timeout = 10000;
    if (request->data) {
        err_ = wmx3Lib_.CreateDevice(request->path.c_str(), DeviceType::DeviceTypeNormal, timeout);
        wmx3Lib_.SetDeviceName(request->name.c_str());
        if (err_ != ErrorCode::None) {
            wmx3Lib_.ErrorToString(err_, errString_, sizeof(errString_));
            snprintf(buffer_, sizeof(buffer_),
                     "Failed to create device. Error=%d (%s)", err_, errString_);
            RCLCPP_ERROR(this->get_logger(), "%s", buffer_);
            response->success = false;
            response->message = std::string(buffer_);
        } else {
            snprintf(buffer_, sizeof(buffer_),
                     "Created device with name: %s", request->name.c_str());
            RCLCPP_INFO(this->get_logger(), "%s", buffer_);
            response->success = true;
            response->message = std::string(buffer_);
        }
    } else {
        err_ = wmx3Lib_.CloseDevice();
        if (err_ != ErrorCode::None) {
            wmx3Lib_.ErrorToString(err_, errString_, sizeof(errString_));
            snprintf(buffer_, sizeof(buffer_),
                     "Failed to close device. Error=%d (%s)", err_, errString_);
            RCLCPP_ERROR(this->get_logger(), "%s", buffer_);
            response->success = false;
            response->message = std::string(buffer_);
        } else {
            commStarted_ = false;
            snprintf(buffer_, sizeof(buffer_), "Device closed");
            RCLCPP_INFO(this->get_logger(), "%s", buffer_);
            response->success = true;
            response->message = std::string(buffer_);
        }
    }
}

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<WmxEngineNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
