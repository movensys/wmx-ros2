#include "wmx_header/wmx_ros2_engine.hpp"

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<WmxRos2Engine>());
    rclcpp::shutdown();
    return 0;
}
