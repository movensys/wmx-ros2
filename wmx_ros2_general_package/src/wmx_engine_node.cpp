#include "wmx_header/engine.hpp"

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<WmxEngine>());
    rclcpp::shutdown();
    return 0;
}
