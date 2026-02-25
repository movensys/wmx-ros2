#include "wmx_ethercat_node.hpp"

WmxEtherCatNode::WmxEtherCatNode() : Node("wmx_ethercat_node"), wmxEcat_(&wmx3Lib_) {

    engineReadySub_ = this->create_subscription<std_msgs::msg::Bool>(
        "/wmx/engine/ready", 1,
        std::bind(&WmxEtherCatNode::onEngineReady, this, _1));

    // Register services immediately — callers receive "not initialized" until engine is up
    getMasterInfoListService_ = this->create_service<wmx_ros2_message::srv::EcatGetMasterInfoList>(
        "/wmx/ecat/get_master_info_list",
        std::bind(&WmxEtherCatNode::getMasterInfoList, this, _1, _2));

    getMasterInfoService_ = this->create_service<wmx_ros2_message::srv::EcatGetMasterInfo>(
        "/wmx/ecat/get_master_info",
        std::bind(&WmxEtherCatNode::getMasterInfo, this, _1, _2));

    changeSlaveStateService_ = this->create_service<wmx_ros2_message::srv::EcatChangeSlaveState>(
        "/wmx/ecat/change_slave_state",
        std::bind(&WmxEtherCatNode::changeSlaveState, this, _1, _2));

    sdoDownloadService_ = this->create_service<wmx_ros2_message::srv::EcatSdoDownload>(
        "/wmx/ecat/sdo_download",
        std::bind(&WmxEtherCatNode::sdoDownload, this, _1, _2));

    sdoUploadService_ = this->create_service<wmx_ros2_message::srv::EcatSdoUpload>(
        "/wmx/ecat/sdo_upload",
        std::bind(&WmxEtherCatNode::sdoUpload, this, _1, _2));

    pdoReadService_ = this->create_service<wmx_ros2_message::srv::EcatPdoRead>(
        "/wmx/ecat/pdo_read",
        std::bind(&WmxEtherCatNode::pdoRead, this, _1, _2));

    txPdoWriteService_ = this->create_service<wmx_ros2_message::srv::EcatTxPdoWrite>(
        "/wmx/ecat/txpdo_write",
        std::bind(&WmxEtherCatNode::txPdoWrite, this, _1, _2));

    diagnosisScanService_ = this->create_service<wmx_ros2_message::srv::EcatDiagnosisScan>(
        "/wmx/ecat/diagnosis_scan",
        std::bind(&WmxEtherCatNode::diagnosisScan, this, _1, _2));

    masterInfoPub_ = this->create_publisher<std_msgs::msg::String>("/wmx/ecat/master_info", 1);
    diagPub_ = this->create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/diagnostics", 1);

    RCLCPP_INFO(this->get_logger(), "wmx_ethercat_node waiting for engine...");
}

WmxEtherCatNode::~WmxEtherCatNode() {
    if (masterInfoTimer_) {
        masterInfoTimer_->cancel();
    }
    if (initialized_) {
        err_ = wmx3Lib_.CloseDevice();
        if (err_ != ErrorCode::None) {
            wmx3Lib_.ErrorToString(err_, errString_, sizeof(errString_));
            RCLCPP_ERROR(this->get_logger(), "Failed to close device");
        } else {
            RCLCPP_INFO(this->get_logger(), "Device closed");
        }
    }
    RCLCPP_INFO(this->get_logger(), "wmx_ethercat_node stopped");
}

void WmxEtherCatNode::onEngineReady(const std_msgs::msg::Bool::SharedPtr msg) {
    if (!msg->data || initialized_) {
        return;
    }

    RCLCPP_INFO(this->get_logger(), "Engine ready — initializing EtherCAT node...");

    unsigned int timeout = 10000;
    err_ = wmx3Lib_.CreateDevice("/opt/lmx/", DeviceType::DeviceTypeNormal, timeout);
    wmx3Lib_.SetDeviceName("wmx_ethercat_node");

    if (err_ != ErrorCode::None) {
        wmx3Lib_.ErrorToString(err_, errString_, sizeof(errString_));
        RCLCPP_ERROR(this->get_logger(),
                     "Failed to attach to device. Error=%d (%s)", err_, errString_);
        return;
    }

    RCLCPP_INFO(this->get_logger(), "Attached to WMX3 device");

    initialized_ = true;

    masterInfoTimer_ = this->create_wall_timer(
        std::chrono::milliseconds(1000),
        std::bind(&WmxEtherCatNode::publishMasterInfo, this));

    // Unsubscribe from ready topic — no longer needed
    engineReadySub_.reset();

    RCLCPP_INFO(this->get_logger(), "wmx_ethercat_node is ready");
}

void WmxEtherCatNode::publishMasterInfo() {
    wmx3Api::ecApi::EcMasterInfo info;
    err_ = wmxEcat_.GetMasterInfo(0, &info);
    if (err_ != ErrorCode::None) {
        return;
    }

    const char* state_str = "Unknown";
    switch (info.state) {
        case wmx3Api::ecApi::EcStateMachine::None:   state_str = "None";   break;
        case wmx3Api::ecApi::EcStateMachine::Init:   state_str = "Init";   break;
        case wmx3Api::ecApi::EcStateMachine::Preop:  state_str = "Preop";  break;
        case wmx3Api::ecApi::EcStateMachine::Boot:   state_str = "Boot";   break;
        case wmx3Api::ecApi::EcStateMachine::Safeop: state_str = "Safeop"; break;
        case wmx3Api::ecApi::EcStateMachine::Op:     state_str = "Op";     break;
        default: break;
    }

    snprintf(buffer_, sizeof(buffer_),
             "id=%d state=%s slaves=%u online=%d offline=%d inaccessible=%d",
             info.id, state_str, info.numOfSlaves,
             info.GetOnlineSlaveCount(),
             info.GetOfflineSlaveCount(),
             info.GetInaccessibleSlaveCount());

    auto msg = std_msgs::msg::String();
    msg.data = std::string(buffer_);
    masterInfoPub_->publish(msg);
}

void WmxEtherCatNode::getMasterInfoList(
    const std::shared_ptr<wmx_ros2_message::srv::EcatGetMasterInfoList::Request> /*request*/,
    std::shared_ptr<wmx_ros2_message::srv::EcatGetMasterInfoList::Response> response) {

    if (!initialized_) {
        response->success = false;
        response->message = "EtherCAT node not initialized. Engine not ready.";
        return;
    }

    wmx3Api::ecApi::EcMasterInfoList list;
    err_ = wmxEcat_.GetMasterInfoList(&list);

    if (err_ != ErrorCode::None) {
        char ecErrString[256];
        wmx3Api::ecApi::Ecat::ErrorToString(err_, ecErrString, sizeof(ecErrString));
        snprintf(buffer_, sizeof(buffer_),
                 "GetMasterInfoList failed. Error=%d (%s)", err_, ecErrString);
        RCLCPP_ERROR(this->get_logger(), "%s", buffer_);
        response->success = false;
        response->message = std::string(buffer_);
        return;
    }

    snprintf(buffer_, sizeof(buffer_),
             "Found %u master(s)", list.numOfMasters);
    RCLCPP_INFO(this->get_logger(), "%s", buffer_);
    response->success = true;
    response->master_count = static_cast<int32_t>(list.numOfMasters);
    response->message = std::string(buffer_);
}

void WmxEtherCatNode::getMasterInfo(
    const std::shared_ptr<wmx_ros2_message::srv::EcatGetMasterInfo::Request> request,
    std::shared_ptr<wmx_ros2_message::srv::EcatGetMasterInfo::Response> response) {

    if (!initialized_) {
        response->success = false;
        response->message = "EtherCAT node not initialized. Engine not ready.";
        return;
    }

    wmx3Api::ecApi::EcMasterInfo info;
    err_ = wmxEcat_.GetMasterInfo(request->master_id, &info);

    if (err_ != ErrorCode::None) {
        char ecErrString[256];
        wmx3Api::ecApi::Ecat::ErrorToString(err_, ecErrString, sizeof(ecErrString));
        snprintf(buffer_, sizeof(buffer_),
                 "GetMasterInfo failed. masterId=%d Error=%d (%s)",
                 request->master_id, err_, ecErrString);
        RCLCPP_ERROR(this->get_logger(), "%s", buffer_);
        response->success = false;
        response->message = std::string(buffer_);
        return;
    }

    const char* state_str = "Unknown";
    switch (info.state) {
        case wmx3Api::ecApi::EcStateMachine::None:   state_str = "None";   break;
        case wmx3Api::ecApi::EcStateMachine::Init:   state_str = "Init";   break;
        case wmx3Api::ecApi::EcStateMachine::Preop:  state_str = "Preop";  break;
        case wmx3Api::ecApi::EcStateMachine::Boot:   state_str = "Boot";   break;
        case wmx3Api::ecApi::EcStateMachine::Safeop: state_str = "Safeop"; break;
        case wmx3Api::ecApi::EcStateMachine::Op:     state_str = "Op";     break;
        default: break;
    }

    response->success = true;
    response->id = info.id;
    response->state = static_cast<int32_t>(info.state);
    response->num_of_slaves = static_cast<int32_t>(info.numOfSlaves);
    response->online_slaves = info.GetOnlineSlaveCount();
    response->offline_slaves = info.GetOfflineSlaveCount();
    response->inaccessible_slaves = info.GetInaccessibleSlaveCount();

    snprintf(buffer_, sizeof(buffer_),
             "Master %d: state=%s slaves=%u online=%d offline=%d inaccessible=%d",
             info.id, state_str, info.numOfSlaves,
             response->online_slaves, response->offline_slaves, response->inaccessible_slaves);
    response->message = std::string(buffer_);
    RCLCPP_INFO(this->get_logger(), "%s", buffer_);
}

void WmxEtherCatNode::changeSlaveState(
    const std::shared_ptr<wmx_ros2_message::srv::EcatChangeSlaveState::Request> request,
    std::shared_ptr<wmx_ros2_message::srv::EcatChangeSlaveState::Response> response) {

    if (!initialized_) {
        response->success = false;
        response->message = "EtherCAT node not initialized. Engine not ready.";
        return;
    }

    int ecErrCode = 0;
    err_ = wmxEcat_.ChangeSlaveState(
        request->master_id,
        request->slave_id,
        static_cast<wmx3Api::ecApi::EcStateMachine::T>(request->state),
        &ecErrCode);

    if (err_ != ErrorCode::None) {
        char ecErrString[256];
        wmx3Api::ecApi::Ecat::ErrorToString(err_, ecErrString, sizeof(ecErrString));
        snprintf(buffer_, sizeof(buffer_),
                 "ChangeSlaveState failed. masterId=%d slaveId=%d state=%d Error=%d (%s) ecError=%d",
                 request->master_id, request->slave_id, request->state, err_, ecErrString, ecErrCode);
        RCLCPP_ERROR(this->get_logger(), "%s", buffer_);
        response->success = false;
        response->error_code = ecErrCode;
        response->message = std::string(buffer_);
        return;
    }

    snprintf(buffer_, sizeof(buffer_),
             "Slave %d state changed to %d on master %d",
             request->slave_id, request->state, request->master_id);
    RCLCPP_INFO(this->get_logger(), "%s", buffer_);
    response->success = true;
    response->error_code = 0;
    response->message = std::string(buffer_);
}

void WmxEtherCatNode::sdoDownload(
    const std::shared_ptr<wmx_ros2_message::srv::EcatSdoDownload::Request> request,
    std::shared_ptr<wmx_ros2_message::srv::EcatSdoDownload::Response> response) {

    if (!initialized_) {
        response->success = false;
        response->message = "EtherCAT node not initialized. Engine not ready.";
        return;
    }

    if (request->data.empty()) {
        response->success = false;
        response->message = "No data provided for SDO download.";
        return;
    }

    std::vector<unsigned char> rawData(request->data.begin(), request->data.end());
    unsigned int sdoErrCode = 0;

    err_ = wmxEcat_.SdoDownload(
        request->master_id,
        request->slave_id,
        request->index,
        request->subindex,
        static_cast<int>(rawData.size()),
        rawData.data(),
        &sdoErrCode);

    if (err_ != ErrorCode::None) {
        char ecErrString[256];
        wmx3Api::ecApi::Ecat::ErrorToString(err_, ecErrString, sizeof(ecErrString));
        snprintf(buffer_, sizeof(buffer_),
                 "SdoDownload failed. slaveId=%d index=0x%04X subindex=%d Error=%d (%s) sdoErr=0x%08X",
                 request->slave_id, request->index, request->subindex,
                 err_, ecErrString, sdoErrCode);
        RCLCPP_ERROR(this->get_logger(), "%s", buffer_);
        response->success = false;
        response->message = std::string(buffer_);
        return;
    }

    snprintf(buffer_, sizeof(buffer_),
             "SdoDownload success. slaveId=%d index=0x%04X subindex=%d bytes=%zu",
             request->slave_id, request->index, request->subindex, rawData.size());
    RCLCPP_INFO(this->get_logger(), "%s", buffer_);
    response->success = true;
    response->message = std::string(buffer_);
}

void WmxEtherCatNode::sdoUpload(
    const std::shared_ptr<wmx_ros2_message::srv::EcatSdoUpload::Request> request,
    std::shared_ptr<wmx_ros2_message::srv::EcatSdoUpload::Response> response) {

    if (!initialized_) {
        response->success = false;
        response->message = "EtherCAT node not initialized. Engine not ready.";
        return;
    }

    if (request->length <= 0) {
        response->success = false;
        response->message = "Invalid length: must be > 0.";
        return;
    }

    std::vector<unsigned char> buff(request->length, 0);
    unsigned int actualSize = 0;
    unsigned int sdoErrCode = 0;

    err_ = wmxEcat_.SdoUpload(
        request->master_id,
        request->slave_id,
        request->index,
        request->subindex,
        request->length,
        buff.data(),
        &actualSize,
        &sdoErrCode);

    if (err_ != ErrorCode::None) {
        char ecErrString[256];
        wmx3Api::ecApi::Ecat::ErrorToString(err_, ecErrString, sizeof(ecErrString));
        snprintf(buffer_, sizeof(buffer_),
                 "SdoUpload failed. slaveId=%d index=0x%04X subindex=%d Error=%d (%s) sdoErr=0x%08X",
                 request->slave_id, request->index, request->subindex,
                 err_, ecErrString, sdoErrCode);
        RCLCPP_ERROR(this->get_logger(), "%s", buffer_);
        response->success = false;
        response->message = std::string(buffer_);
        return;
    }

    response->success = true;
    response->actual_size = static_cast<int32_t>(actualSize);
    response->data.assign(buff.begin(), buff.begin() + actualSize);

    snprintf(buffer_, sizeof(buffer_),
             "SdoUpload success. slaveId=%d index=0x%04X subindex=%d actualSize=%u",
             request->slave_id, request->index, request->subindex, actualSize);
    RCLCPP_INFO(this->get_logger(), "%s", buffer_);
    response->message = std::string(buffer_);
}

void WmxEtherCatNode::pdoRead(
    const std::shared_ptr<wmx_ros2_message::srv::EcatPdoRead::Request> request,
    std::shared_ptr<wmx_ros2_message::srv::EcatPdoRead::Response> response) {

    if (!initialized_) {
        response->success = false;
        response->message = "EtherCAT node not initialized. Engine not ready.";
        return;
    }

    if (request->length <= 0) {
        response->success = false;
        response->message = "Invalid length: must be > 0.";
        return;
    }

    std::vector<unsigned char> buff(request->length, 0);
    unsigned int actualSize = 0;

    err_ = wmxEcat_.PdoRead(
        request->master_id,
        request->slave_id,
        request->index,
        request->subindex,
        request->length,
        buff.data(),
        &actualSize);

    if (err_ != ErrorCode::None) {
        char ecErrString[256];
        wmx3Api::ecApi::Ecat::ErrorToString(err_, ecErrString, sizeof(ecErrString));
        snprintf(buffer_, sizeof(buffer_),
                 "PdoRead failed. slaveId=%d index=0x%04X subindex=%d Error=%d (%s)",
                 request->slave_id, request->index, request->subindex, err_, ecErrString);
        RCLCPP_ERROR(this->get_logger(), "%s", buffer_);
        response->success = false;
        response->message = std::string(buffer_);
        return;
    }

    response->success = true;
    response->actual_size = static_cast<int32_t>(actualSize);
    response->data.assign(buff.begin(), buff.begin() + actualSize);

    snprintf(buffer_, sizeof(buffer_),
             "PdoRead success. slaveId=%d index=0x%04X subindex=%d actualSize=%u",
             request->slave_id, request->index, request->subindex, actualSize);
    RCLCPP_INFO(this->get_logger(), "%s", buffer_);
    response->message = std::string(buffer_);
}

void WmxEtherCatNode::txPdoWrite(
    const std::shared_ptr<wmx_ros2_message::srv::EcatTxPdoWrite::Request> request,
    std::shared_ptr<wmx_ros2_message::srv::EcatTxPdoWrite::Response> response) {

    if (!initialized_) {
        response->success = false;
        response->message = "EtherCAT node not initialized. Engine not ready.";
        return;
    }

    if (request->data.empty()) {
        response->success = false;
        response->message = "No data provided for TxPDO write.";
        return;
    }

    std::vector<unsigned char> rawData(request->data.begin(), request->data.end());

    err_ = wmxEcat_.TxPdoWrite(
        request->master_id,
        request->slave_id,
        request->index,
        request->subindex,
        static_cast<int>(rawData.size()),
        rawData.data());

    if (err_ != ErrorCode::None) {
        char ecErrString[256];
        wmx3Api::ecApi::Ecat::ErrorToString(err_, ecErrString, sizeof(ecErrString));
        snprintf(buffer_, sizeof(buffer_),
                 "TxPdoWrite failed. slaveId=%d index=0x%04X subindex=%d Error=%d (%s)",
                 request->slave_id, request->index, request->subindex, err_, ecErrString);
        RCLCPP_ERROR(this->get_logger(), "%s", buffer_);
        response->success = false;
        response->message = std::string(buffer_);
        return;
    }

    snprintf(buffer_, sizeof(buffer_),
             "TxPdoWrite success. slaveId=%d index=0x%04X subindex=%d bytes=%zu",
             request->slave_id, request->index, request->subindex, rawData.size());
    RCLCPP_INFO(this->get_logger(), "%s", buffer_);
    response->success = true;
    response->message = std::string(buffer_);
}

void WmxEtherCatNode::diagnosisScan(
    const std::shared_ptr<wmx_ros2_message::srv::EcatDiagnosisScan::Request> request,
    std::shared_ptr<wmx_ros2_message::srv::EcatDiagnosisScan::Response> response) {

    if (!initialized_) {
        response->success = false;
        response->message = "EtherCAT node not initialized. Engine not ready.";
        return;
    }

    err_ = wmxEcat_.DiagnosisScan(request->master_id);

    if (err_ != ErrorCode::None) {
        char ecErrString[256];
        wmx3Api::ecApi::Ecat::ErrorToString(err_, ecErrString, sizeof(ecErrString));
        snprintf(buffer_, sizeof(buffer_),
                 "DiagnosisScan failed. masterId=%d Error=%d (%s)",
                 request->master_id, err_, ecErrString);
        RCLCPP_ERROR(this->get_logger(), "%s", buffer_);
        response->success = false;
        response->message = std::string(buffer_);
        return;
    }

    // Read master info to populate diagnostics
    wmx3Api::ecApi::EcMasterInfo info;
    err_ = wmxEcat_.GetMasterInfo(request->master_id, &info);
    if (err_ == ErrorCode::None) {
        auto diagArray = diagnostic_msgs::msg::DiagnosticArray();
        diagArray.header.stamp = this->get_clock()->now();

        for (unsigned int i = 0; i < info.numOfSlaves; ++i) {
            const auto& slave = info.slaves[i];

            auto status = diagnostic_msgs::msg::DiagnosticStatus();
            snprintf(buffer_, sizeof(buffer_), "EtherCAT Slave %d (addr=0x%04X)", slave.id, slave.address);
            status.name = std::string(buffer_);
            status.hardware_id = std::to_string(slave.id);

            // Bit3 of portState is set when a broken connection is detected
            bool anyBroken = false;
            for (int p = 0; p < 4; ++p) {
                if (slave.portState[p] & 0x08) {
                    anyBroken = true;
                }
            }

            if (slave.offline) {
                status.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
                status.message = "Slave offline";
            } else if (slave.inaccessible) {
                status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
                status.message = "Slave inaccessible";
            } else if (anyBroken) {
                status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
                status.message = "Broken port detected";
            } else {
                status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
                status.message = "OK";
            }

            auto kv = diagnostic_msgs::msg::KeyValue();
            kv.key = "state";
            kv.value = std::to_string(static_cast<int>(slave.state));
            status.values.push_back(kv);

            kv.key = "vendorId";
            snprintf(buffer_, sizeof(buffer_), "0x%08X", slave.vendorId);
            kv.value = std::string(buffer_);
            status.values.push_back(kv);

            kv.key = "productCode";
            snprintf(buffer_, sizeof(buffer_), "0x%08X", slave.productCode);
            kv.value = std::string(buffer_);
            status.values.push_back(kv);

            diagArray.status.push_back(status);
        }

        diagPub_->publish(diagArray);
        RCLCPP_INFO(this->get_logger(),
                    "DiagnosisScan published diagnostics for %u slaves", info.numOfSlaves);
    }

    response->success = true;
    snprintf(buffer_, sizeof(buffer_),
             "DiagnosisScan complete. masterId=%d", request->master_id);
    response->message = std::string(buffer_);
}

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<WmxEtherCatNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
