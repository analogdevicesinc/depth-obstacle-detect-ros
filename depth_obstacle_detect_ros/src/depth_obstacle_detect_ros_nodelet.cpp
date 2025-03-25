#include "depth_obstacle_detect_ros.h"
#include "nodelet/nodelet.h"
#include "pluginlib/class_list_macros.h"
#include <memory>

namespace obstacle_detect
{
class ObstacleDetect : public nodelet::Nodelet
{
private:
  std::unique_ptr<DepthObstacleDetect> Obj;

public:
  virtual void onInit()
  {
    ros::NodeHandle& nh = getPrivateNodeHandle();
    Obj = std::make_unique<DepthObstacleDetect>(nh);
  }
};
}  // namespace obstacle_detect

PLUGINLIB_EXPORT_CLASS(obstacle_detect::ObstacleDetect, nodelet::Nodelet);