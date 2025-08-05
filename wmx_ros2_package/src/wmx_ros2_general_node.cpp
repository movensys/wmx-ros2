#include "wmx_general_header/wmx_ros2_general.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<WmxRos2General>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
