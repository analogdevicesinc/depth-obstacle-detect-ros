#include "depth_obstacle_detect_ros.h"

DepthObstacleDetect::DepthObstacleDetect(ros::NodeHandle& node) : nh_(node)
{
  image_transport::ImageTransport it(nh_);

  // Get topic names from launch file parameters
  std::string depth_topic;
  std::string camera_info_topic;
  std::string detect_topic;
  std::string obstacle_state_topic;
  std::string cam_id;
  nh_.param<std::string>("depth_topic", depth_topic, "tof_cam/rect/depth");
  nh_.param<std::string>("camera_info_topic", camera_info_topic, "tof_cam/rect/camera_info");
  nh_.param<std::string>("detect_topic", detect_topic, "tof_cam/rect/detect");
  nh_.param<std::string>("obstacle_state_topic", obstacle_state_topic, "tof_cam/obstacle_state");
  nh_.param<std::string>("cam_id", cam_id, "tof_cam");
  nh_.param("obstacle_range_limit", obstacle_range_, 1.0);
  obstacle_range_threshold_ = obstacle_range_ * 1000.0;
  nh_.param("width_regions", width_steps_, 4);
  nh_.param("height_regions", height_steps_, 4);
  nh_.param("verbose", verbose_, false);
  nh_.param("max_distance_for_visualization", max_distance_, 16);

  init_distance_conversion_ = true;
  sub_depth_ = it.subscribe(depth_topic, 1, &DepthObstacleDetect::imageCallback, this);
  sub_info_ = nh_.subscribe(camera_info_topic, 1, &DepthObstacleDetect::infoCallback, this);
  pub_detect_ = it.advertise(detect_topic, 1);
  pub_obstacle_ = nh_.advertise<depth_obstacle_detect_ros_msgs::ObstacleStampedArray>(obstacle_state_topic, 1);

  msg_header_detect_.frame_id = cam_id;
  msg_header_detect_.stamp = ros::Time::now();

  msg_header_obstacle_.frame_id = cam_id;
  msg_header_obstacle_.stamp = ros::Time::now();
  msg_obstacle_.header = msg_header_obstacle_;

  ROS_INFO("##### Image Width Regions  = %d, Image Height Regions  = %d. #####", width_steps_, height_steps_);
  ROS_INFO("##### Obstacle Range Limit = %fm (or %fmm). #####", obstacle_range_, obstacle_range_threshold_);
}

void DepthObstacleDetect::imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  try
  {
    // Convert Image message to cv::Mat
    if (msg->encoding == "mono16" || msg->encoding == "16UC1")
    {
      detect_image_ = cv_bridge::toCvCopy(msg, "16UC1")->image;
      is32FC1_ = false;
      if (verbose_)
        ROS_INFO("Detected 16UC1 Image");
    }
    else if (msg->encoding == "32FC1")
    {
      detect_image_ = cv_bridge::toCvCopy(msg, "32FC1")->image;
      is32FC1_ = true;
      if (init_distance_conversion_)
      {
        init_distance_conversion_ = false;
        obstacle_range_threshold_ /= 1000;
      }
      if (verbose_)
        ROS_INFO("Detected 32FC1 Image");
    }
    else
    {
      throw cv::Error::StsUnsupportedFormat;
    }
    // Get Image Size
    image_height_ = detect_image_.rows;
    image_width_ = detect_image_.cols;
    region_width_ = int(image_width_ / width_steps_);
    region_height_ = int(image_height_ / height_steps_);

    // Scale factor is used to convert the printed distance to be in m or mm
    double scale_factor = (is32FC1_) ? 1.0 : 1000.0;
    double pixel_scale_factor = 255.0/(max_distance_*scale_factor);
    // ROS_INFO("Image Width  = %d, Image Height  = %d.",  image_width_,  image_height_);
    // ROS_INFO("Width Steps  = %d, Height Steps  = %d.",  width_steps_,  height_steps_);
    // ROS_INFO("Region Width = %d, Region Height = %d.", region_width_, region_height_);

    // Perpare Output Image - normalize and covert input depth image to RGB grayscale image.
    // cv::normalize(detect_image_, detect_image_8bit_, 0, 255, cv::NORM_MINMAX, CV_8UC1);
    detect_image_.convertTo(detect_image_8bit_, CV_8UC1, pixel_scale_factor);
    cv::cvtColor(detect_image_8bit_, detect_image_out_, CV_GRAY2RGB);

    // Detect black (value=0) border area due to fov limiting and replace pixels with the highest values in the image.
    // This prevents the border areas from generating false obstacles triggers.
    cv::minMaxIdx(detect_image_, &minValue_, &maxValue_);  // Find the min and max pixel values in the image.
    image_maxValue_ = int(maxValue_);
    cv::inRange(detect_image_, 0, 0, fov_border_mask_);      // Create the border mask based on 0 value pixels/
    detect_image_.setTo(image_maxValue_, fov_border_mask_);  // Use the mask to set all of the border pixel values to
                                                             // the max image pixel value.

    /* Use a Gaussian Filter to reduce depth image noise. */
    cv::GaussianBlur(detect_image_, detect_image_blur_, cv::Size(3, 3), 0, 0);
    /* Uncomment next line to bypass Gaussian Filter step. */
    // detect_image_blur_ = detect_image_;

    // Clear Obstacle Message Arrays.
    msg_obstacle_.obstacle_detected.clear();
    msg_obstacle_.obstacle_distance.clear();

    // Check each depth image region for an obstacle and highlight in output image.
    char buffer[5];
    for (int i = 0; i < width_steps_; i++)
    {
      for (int j = 0; j < height_steps_; j++)
      {
        x_step_ = i * region_width_;
        y_step_ = j * region_height_;
        cv::Rect rect(x_step_, y_step_, region_width_, region_height_);
        cv::minMaxIdx(detect_image_blur_(rect), &minValue_, &maxValue_);
        if (verbose_)
          ROS_INFO("%f", minValue_);
        if (minValue_ >= obstacle_range_threshold_)
        {
          cv::rectangle(detect_image_out_, rect, cv::Scalar(0, 255, 0));
          sprintf(buffer, "%3.1f", minValue_ / scale_factor);
          std::string overlay_string(buffer);
          cv::putText(detect_image_out_, overlay_string,
                      cv::Point(x_step_ + int(region_width_ / 6), y_step_ + int(region_height_ / 2) + 2),
                      cv::FONT_HERSHEY_DUPLEX, 0.5, CV_RGB(0, 255, 0), 1);
          obstacle_detected_ = false;
          msg_obstacle_.obstacle_detected.push_back(obstacle_detected_);
          obstacle_detected_range_ = minValue_ / scale_factor;
          msg_obstacle_.obstacle_distance.push_back(obstacle_detected_range_);
        }
        else
        {
          cv::rectangle(detect_image_out_, rect, cv::Scalar(255, 0, 0));
          sprintf(buffer, "%3.1f", minValue_ / scale_factor);
          std::string overlay_string(buffer);
          cv::putText(detect_image_out_, overlay_string,
                      cv::Point(x_step_ + int(region_width_ / 6), y_step_ + int(region_height_ / 2) + 2),
                      cv::FONT_HERSHEY_DUPLEX, 0.5, CV_RGB(0, 0, 255), 1);
          obstacle_detected_ = true;
          msg_obstacle_.obstacle_detected.push_back(obstacle_detected_);
          obstacle_detected_range_ = minValue_ / scale_factor;
          msg_obstacle_.obstacle_distance.push_back(obstacle_detected_range_);
          if (verbose_)
            ROS_INFO("Obstacle Detected in Region (%d, %d), Range = %fm.", i, j, minValue_ / 1000.0);
        }
      }
    }

    // Publish output Image and obstacle state message.
    msg_header_detect_.stamp = ros::Time::now();
    sensor_msgs::ImagePtr msg_detect = cv_bridge::CvImage(msg_header_detect_, "rgb8", detect_image_out_).toImageMsg();
    pub_detect_.publish(msg_detect);
    msg_obstacle_.header.stamp = msg_header_detect_.stamp;
    pub_obstacle_.publish(msg_obstacle_);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Error in Callback OpenCV processing.");
  }
}

void DepthObstacleDetect::infoCallback(const sensor_msgs::CameraInfo msg)
{
  cam_info_detect_ = msg;
  // Do nothing else - may be used to add future functionality.
}