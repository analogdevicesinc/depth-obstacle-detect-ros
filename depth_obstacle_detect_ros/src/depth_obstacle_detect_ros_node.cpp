#include "depth_obstacle_detect_ros.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "depth_obstacle_detect_ros_node");
  ros::NodeHandle nh;
  DepthObstacleDetect ObstacleDetectObject(nh);
  ros::spin();
  return 0;
}
