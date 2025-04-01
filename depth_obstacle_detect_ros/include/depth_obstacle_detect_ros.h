#ifndef DEPTH_OBSTACLE_DETECT_ROS_H
#define DEPTH_OBSTACLE_DETECT_ROS_H
#include <string>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#ifdef OPENCV2
#include <opencv2/contrib/contrib.hpp>
#endif

#include <depth_obstacle_detect_ros_msgs/ObstacleStampedArray.h>
#include "std_msgs/Bool.h"
#include "std_msgs/Float64.h"

class DepthObstacleDetect
{
public:
  DepthObstacleDetect(ros::NodeHandle&);
  void imageCallback(const sensor_msgs::ImageConstPtr&);
  void infoCallback(const sensor_msgs::CameraInfo);

private:
  ros::NodeHandle& nh_;
  ros::Subscriber sub_info_;
  ros::Publisher pub_obstacle_;
  image_transport::Subscriber sub_depth_;
  image_transport::Publisher pub_detect_;
  sensor_msgs::CameraInfo cam_info_detect_;
  std_msgs::Header msg_header_detect_;
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
  depth_obstacle_detect_ros_msgs::ObstacleStampedArray msg_obstacle_;
  std_msgs::Header msg_header_obstacle_;
  bool obstacle_detected_;
  double obstacle_detected_range_;
};

#endif