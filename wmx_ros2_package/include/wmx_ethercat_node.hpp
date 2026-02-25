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
#include "std_msgs/msg/string.hpp"
#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"

#include "wmx_ros2_message/srv/ecat_get_master_info_list.hpp"
#include "wmx_ros2_message/srv/ecat_get_master_info.hpp"
#include "wmx_ros2_message/srv/ecat_change_slave_state.hpp"
#include "wmx_ros2_message/srv/ecat_sdo_download.hpp"
#include "wmx_ros2_message/srv/ecat_sdo_upload.hpp"
#include "wmx_ros2_message/srv/ecat_pdo_read.hpp"
#include "wmx_ros2_message/srv/ecat_tx_pdo_write.hpp"
#include "wmx_ros2_message/srv/ecat_diagnosis_scan.hpp"

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

    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr engineReadySub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr masterInfoPub_;
    rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagPub_;
    rclcpp::TimerBase::SharedPtr masterInfoTimer_;

    rclcpp::Service<wmx_ros2_message::srv::EcatGetMasterInfoList>::SharedPtr getMasterInfoListService_;
    rclcpp::Service<wmx_ros2_message::srv::EcatGetMasterInfo>::SharedPtr getMasterInfoService_;
    rclcpp::Service<wmx_ros2_message::srv::EcatChangeSlaveState>::SharedPtr changeSlaveStateService_;
    rclcpp::Service<wmx_ros2_message::srv::EcatSdoDownload>::SharedPtr sdoDownloadService_;
    rclcpp::Service<wmx_ros2_message::srv::EcatSdoUpload>::SharedPtr sdoUploadService_;
    rclcpp::Service<wmx_ros2_message::srv::EcatPdoRead>::SharedPtr pdoReadService_;
    rclcpp::Service<wmx_ros2_message::srv::EcatTxPdoWrite>::SharedPtr txPdoWriteService_;
    rclcpp::Service<wmx_ros2_message::srv::EcatDiagnosisScan>::SharedPtr diagnosisScanService_;

    void onEngineReady(const std_msgs::msg::Bool::SharedPtr msg);
    void publishMasterInfo();

    void getMasterInfoList(
        const std::shared_ptr<wmx_ros2_message::srv::EcatGetMasterInfoList::Request> request,
        std::shared_ptr<wmx_ros2_message::srv::EcatGetMasterInfoList::Response> response);

    void getMasterInfo(
        const std::shared_ptr<wmx_ros2_message::srv::EcatGetMasterInfo::Request> request,
        std::shared_ptr<wmx_ros2_message::srv::EcatGetMasterInfo::Response> response);

    void changeSlaveState(
        const std::shared_ptr<wmx_ros2_message::srv::EcatChangeSlaveState::Request> request,
        std::shared_ptr<wmx_ros2_message::srv::EcatChangeSlaveState::Response> response);

    void sdoDownload(
        const std::shared_ptr<wmx_ros2_message::srv::EcatSdoDownload::Request> request,
        std::shared_ptr<wmx_ros2_message::srv::EcatSdoDownload::Response> response);

    void sdoUpload(
        const std::shared_ptr<wmx_ros2_message::srv::EcatSdoUpload::Request> request,
        std::shared_ptr<wmx_ros2_message::srv::EcatSdoUpload::Response> response);

    void pdoRead(
        const std::shared_ptr<wmx_ros2_message::srv::EcatPdoRead::Request> request,
        std::shared_ptr<wmx_ros2_message::srv::EcatPdoRead::Response> response);

    void txPdoWrite(
        const std::shared_ptr<wmx_ros2_message::srv::EcatTxPdoWrite::Request> request,
        std::shared_ptr<wmx_ros2_message::srv::EcatTxPdoWrite::Response> response);

    void diagnosisScan(
        const std::shared_ptr<wmx_ros2_message::srv::EcatDiagnosisScan::Request> request,
        std::shared_ptr<wmx_ros2_message::srv::EcatDiagnosisScan::Response> response);
};

#endif  // WMX_ETHERCAT_NODE_HPP
