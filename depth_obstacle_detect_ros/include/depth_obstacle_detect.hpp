#ifndef DEPTH_OBSTACLE_DETECT_H
#define DEPTH_OBSTACLE_DETECT_H
#include <string>
#include <rclcpp/rclcpp.hpp>
#include <image_transport/image_transport.hpp>
#ifdef ROS_HUMBLE
#include <cv_bridge/cv_bridge.h>
#else
#include <cv_bridge/cv_bridge.hpp>
#endif
#include <sensor_msgs/image_encodings.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#ifdef OPENCV2
#include <opencv2/contrib/contrib.hpp>
#endif

#include "depth_obstacle_detect_ros_msgs/msg/obstacle_stamped_array.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float64.hpp"

class DepthObstacleDetect : public rclcpp::Node
{
public:
  explicit DepthObstacleDetect(const rclcpp::NodeOptions& options);
  void init();
  void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr&);
  void infoCallback(const sensor_msgs::msg::CameraInfo&);

private:
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr sub_info_;
  rclcpp::Publisher<depth_obstacle_detect_ros_msgs::msg::ObstacleStampedArray>::SharedPtr pub_obstacle_;
  image_transport::Subscriber sub_depth_;
  image_transport::Publisher pub_detect_;
  sensor_msgs::msg::CameraInfo cam_info_detect_;
  std_msgs::msg::Header msg_header_detect_;
  std::string depth_topic_;
  std::string camera_info_topic_;
  std::string detect_topic_;
  std::string obstacle_state_topic_;
  std::string cam_id_;
  cv::Mat detect_image_;
  cv::Mat detect_image_blur_;
  cv::Mat detect_image_8bit_;
  cv::Mat detect_image_out_;
  cv::Mat fov_border_mask_;
  int image_width_;
  int image_height_;
  int width_steps_;
  int height_steps_;
  int region_width_;
  int region_height_;
  int x_step_;
  int y_step_;
  int max_distance_;
  double minValue_, maxValue_;
  int image_maxValue_;
  double obstacle_range_;            // in m
  double obstacle_range_threshold_;  // in mm
  bool verbose_;
  bool is32FC1_;
  bool init_distance_conversion_;
  depth_obstacle_detect_ros_msgs::msg::ObstacleStampedArray msg_obstacle_;
  std_msgs::msg::Header msg_header_obstacle_;
  bool obstacle_detected_;
  double obstacle_detected_range_;
};

#endif