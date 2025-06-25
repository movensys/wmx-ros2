#include "wmx_header/wmx_ros2_axis_pose.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<WmxRos2AxisPoseServer>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
