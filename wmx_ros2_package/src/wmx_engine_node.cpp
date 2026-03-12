#include "wmx_engine_node.hpp"

using std::placeholders::_1;
using std::placeholders::_2;

WmxEngineNode::WmxEngineNode() : Node("wmx_engine_node"), wmx3Lib_Ecat_(&wmx3Lib_) {
    auto ready_qos = rclcpp::QoS(1).reliable().transient_local();
    engineReadyPub_ = this->create_publisher<std_msgs::msg::Bool>("wmx/engine/ready", ready_qos);

    setEngineService_ = this->create_service<wmx_ros2_message::srv::SetEngine>(
        "wmx/engine/set_device",
        std::bind(&WmxEngineNode::setEngine, this, _1, _2));

    setCommService_ = this->create_service<std_srvs::srv::SetBool>(
        "wmx/engine/set_comm",
        std::bind(&WmxEngineNode::setComm, this, _1, _2));

    getEngineStatusService_ = this->create_service<std_srvs::srv::Trigger>(
        "wmx/engine/get_status",
        std::bind(&WmxEngineNode::getEngineStatus, this, _1, _2));

    scanNetworkService_ = this->create_service<std_srvs::srv::Trigger>(
        "wmx/engine/scan_network",
        std::bind(&WmxEngineNode::scanNetwork, this, _1, _2));

    readyTimer_ = this->create_wall_timer(
        std::chrono::milliseconds(1000),
        std::bind(&WmxEngineNode::publishReady, this));

    // Run startEngine on a background thread to avoid blocking the executor
    // during construction. The executor must be spinning so services and
    // timers can fire while the engine initialises.
    startThread_ = std::thread(&WmxEngineNode::startEngine, this);

    RCLCPP_INFO(this->get_logger(), "wmx_engine_node is ready");
}

WmxEngineNode::~WmxEngineNode() {
    if (startThread_.joinable()) {
        startThread_.join();
    }
    stopCommunication();
    stopEngine();
    std::this_thread::sleep_for(std::chrono::seconds(3));
    RCLCPP_INFO(this->get_logger(), "wmx_engine_node stopped");
}

void WmxEngineNode::publishReady() {
    auto msg = std_msgs::msg::Bool();
    msg.data = commStarted_.load();
    engineReadyPub_->publish(msg);
    if (commStarted_.load() && readyTimer_) {
        readyTimer_->cancel();
    }
}

void WmxEngineNode::startEngine() {
    RCLCPP_INFO(this->get_logger(), "Starting engine...");
    unsigned int timeout = 10000;
    int maxRetries = 5;
    int retryDelay = 2000;
    const int CreateDeviceLockError = 297;
    int err;
    char errString[256];

    std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    for (int attempt = 0; attempt < maxRetries; attempt++) {
        if (attempt > 0) {
            RCLCPP_INFO(this->get_logger(), "Retrying device creation (attempt %d/%d)...",
                        attempt + 1, maxRetries);
            std::this_thread::sleep_for(std::chrono::milliseconds(retryDelay));
        }

        err = wmx3Lib_.CreateDevice(WMX3_SDK_PATH, wmx3Api::DeviceType::DeviceTypeNormal, timeout);

        if (err == wmx3Api::ErrorCode::None) {
            wmx3Lib_.SetDeviceName("wmx_engine_node");
            RCLCPP_INFO(this->get_logger(), "Device created (attempt %d)", attempt + 1);

            err = wmx3Lib_.StartCommunication(timeout);
            if (err == wmx3Api::ErrorCode::None) {
                RCLCPP_INFO(this->get_logger(), "Communication started");
                commStarted_ = true;
                startComplete_ = true;
            } else {
                wmx3Lib_.ErrorToString(err, errString, sizeof(errString));
                RCLCPP_ERROR(this->get_logger(),
                             "Failed to start communication. Error=%d (%s)", err, errString);
            }
            startComplete_ = true;
            return;
        } else {
            wmx3Lib_.ErrorToString(err, errString, sizeof(errString));
            if (err == CreateDeviceLockError) {
                RCLCPP_WARN(this->get_logger(),
                            "Device lock error (attempt %d/%d). Waiting...",
                            attempt + 1, maxRetries);
            } else {
                RCLCPP_WARN(this->get_logger(),
                            "Failed to create device (attempt %d/%d). Error=%d (%s)",
                            attempt + 1, maxRetries, err, errString);
            }
        }
    }

    wmx3Lib_.ErrorToString(err, errString, sizeof(errString));
    RCLCPP_ERROR(this->get_logger(),
                 "Failed to create device after %d attempts. Error=%d (%s)",
                 maxRetries, err, errString);
    startComplete_ = true;
}

void WmxEngineNode::stopCommunication() {
    unsigned int timeout = 10000;
    int err;
    char errString[256];
    err = wmx3Lib_.StopCommunication(timeout);
    if (err != wmx3Api::ErrorCode::None) {
        wmx3Lib_.ErrorToString(err, errString, sizeof(errString));
        RCLCPP_ERROR(this->get_logger(), "Failed to stop communication");
    } else {
        RCLCPP_INFO(this->get_logger(), "Communication stopped");
    }
    commStarted_ = false;
}

void WmxEngineNode::stopEngine() {
    int err;
    char errString[256];

    unsigned int timeout = 10000;
    err = wmx3Lib_.StopEngine(timeout);
    if (err != wmx3Api::ErrorCode::None) {
        wmx3Lib_.ErrorToString(err, errString, sizeof(errString));
        RCLCPP_ERROR(this->get_logger(), "Failed to stop engine");
    } else {
        RCLCPP_INFO(this->get_logger(), "Engine stopped");
    }

    err = wmx3Lib_.CloseDevice();
    if (err != wmx3Api::ErrorCode::None) {
        wmx3Lib_.ErrorToString(err, errString, sizeof(errString));
        RCLCPP_ERROR(this->get_logger(), "Failed to close device");
    } else {
        RCLCPP_INFO(this->get_logger(), "Device closed");
    }
}

void WmxEngineNode::getEngineStatus(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response) {

    if (!startComplete_) {
        response->success = false;
        response->message = "Engine startup in progress";
        return;
    }

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

void WmxEngineNode::scanNetwork(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response) {

    if (!startComplete_) {
        response->success = false;
        response->message = "Engine startup in progress";
        return;
    }

    int err;
    char ecErrString[256];
    char buffer[512];
    const int masterId = 0;
    err = wmx3Lib_Ecat_.ScanNetwork(masterId);

    if (err != wmx3Api::ErrorCode::None) {
        wmx3Api::ecApi::Ecat::ErrorToString(err, ecErrString, sizeof(ecErrString));
        snprintf(buffer, sizeof(buffer),
                 "Failed to scan network. Error=%d (%s)", err, ecErrString);
        RCLCPP_ERROR(this->get_logger(), "%s", buffer);
        response->success = false;
        response->message = std::string(buffer);
    } else {
        RCLCPP_INFO(this->get_logger(), "Scan network operation done!");
        response->success = true;
        response->message = "Scan network operation done!";
    }
}

void WmxEngineNode::setComm(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    std::shared_ptr<std_srvs::srv::SetBool::Response> response) {

    if (!startComplete_) {
        response->success = false;
        response->message = "Engine startup in progress";
        return;
    }

    unsigned int timeout = 10000;
    int err;
    char errString[256];
    char buffer[512];
    if (request->data) {
        err = wmx3Lib_.StartCommunication(timeout);
        if (err != wmx3Api::ErrorCode::None) {
            wmx3Lib_.ErrorToString(err, errString, sizeof(errString));
            snprintf(buffer, sizeof(buffer),
                     "Failed to start communication. Error=%d (%s)", err, errString);
            RCLCPP_ERROR(this->get_logger(), "%s", buffer);
            response->success = false;
            response->message = std::string(buffer);
        } else {
            commStarted_ = true;
            publishReady();
            snprintf(buffer, sizeof(buffer), "Communication started");
            RCLCPP_INFO(this->get_logger(), "%s", buffer);
            response->success = true;
            response->message = std::string(buffer);
        }
    } else {
        err = wmx3Lib_.StopCommunication(timeout);
        if (err != wmx3Api::ErrorCode::None) {
            wmx3Lib_.ErrorToString(err, errString, sizeof(errString));
            snprintf(buffer, sizeof(buffer),
                     "Failed to stop communication. Error=%d (%s)", err, errString);
            RCLCPP_ERROR(this->get_logger(), "%s", buffer);
            response->success = false;
            response->message = std::string(buffer);
        } else {
            commStarted_ = false;
            publishReady();
            snprintf(buffer, sizeof(buffer), "Communication stopped");
            RCLCPP_INFO(this->get_logger(), "%s", buffer);
            response->success = true;
            response->message = std::string(buffer);
        }
    }
}

void WmxEngineNode::setEngine(
    const std::shared_ptr<wmx_ros2_message::srv::SetEngine::Request> request,
    std::shared_ptr<wmx_ros2_message::srv::SetEngine::Response> response) {

    if (!startComplete_) {
        response->success = false;
        response->message = "Engine startup in progress";
        return;
    }

    unsigned int timeout = 10000;
    int err;
    char errString[256];
    char buffer[512];
    if (request->data) {
        err = wmx3Lib_.CreateDevice(request->path.c_str(), wmx3Api::DeviceType::DeviceTypeNormal, timeout);
        if (err != wmx3Api::ErrorCode::None) {
            wmx3Lib_.ErrorToString(err, errString, sizeof(errString));
            snprintf(buffer, sizeof(buffer),
                     "Failed to create device. Error=%d (%s)", err, errString);
            RCLCPP_ERROR(this->get_logger(), "%s", buffer);
            response->success = false;
            response->message = std::string(buffer);
        } else {
            wmx3Lib_.SetDeviceName(request->name.c_str());
            snprintf(buffer, sizeof(buffer),
                     "Created device with name: %s", request->name.c_str());
            RCLCPP_INFO(this->get_logger(), "%s", buffer);
            response->success = true;
            response->message = std::string(buffer);
        }
    } else {
        err = wmx3Lib_.CloseDevice();
        if (err != wmx3Api::ErrorCode::None) {
            wmx3Lib_.ErrorToString(err, errString, sizeof(errString));
            snprintf(buffer, sizeof(buffer),
                     "Failed to close device. Error=%d (%s)", err, errString);
            RCLCPP_ERROR(this->get_logger(), "%s", buffer);
            response->success = false;
            response->message = std::string(buffer);
        } else {
            commStarted_ = false;
            snprintf(buffer, sizeof(buffer), "Device closed");
            RCLCPP_INFO(this->get_logger(), "%s", buffer);
            response->success = true;
            response->message = std::string(buffer);
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
