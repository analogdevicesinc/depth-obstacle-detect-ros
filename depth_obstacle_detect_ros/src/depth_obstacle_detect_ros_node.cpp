#include "depth_obstacle_detect.hpp"

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  auto node = std::make_shared<DepthObstacleDetect>(options);
  rclcpp::spin(node);
  return 0;
}
