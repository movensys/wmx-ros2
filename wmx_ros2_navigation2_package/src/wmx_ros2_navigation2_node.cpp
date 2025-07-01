#include "wmx_header/wmx_ros2_navigation2.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<WmxRos2Navigation2>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
