#include "wmx_header/wmx_ros2_core_motion.hpp"

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<WmxRos2CoreMotion>());
    rclcpp::shutdown();
    return 0;
}
