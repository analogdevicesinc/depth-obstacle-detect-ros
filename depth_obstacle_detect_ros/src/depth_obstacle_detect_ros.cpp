#include "depth_obstacle_detect.hpp"
#include <chrono>

DepthObstacleDetect::DepthObstacleDetect(const rclcpp::NodeOptions& options) : Node("obstacle_detect_node", options)
{
  // Reading parameters from the launch file
  this->declare_parameter("depth_topic", "tof_cam/rect/depth");
  this->declare_parameter("detect_topic", "tof_cam/rect/detect");
  this->declare_parameter("cam_id", "tof_cam");
  this->declare_parameter("camera_info_topic", "tof_cam/rect/camera_info");
  this->declare_parameter("verbose", false);
  this->declare_parameter("width_regions", 4);
  this->declare_parameter("height_regions", 4);
  this->declare_parameter("obstacle_range_limit", 1.0);
  this->declare_parameter("max_distance_for_visualization", 16);
  this->declare_parameter("obstacle_state_topic", "tof_cam/obstacle_state");

  this->get_parameter("depth_topic", depth_topic_);
  this->get_parameter("detect_topic", detect_topic_);
  this->get_parameter("cam_id", cam_id_);
  this->get_parameter("verbose", verbose_);
  this->get_parameter("camera_info_topic", camera_info_topic_);
  this->get_parameter("obstacle_range_limit", obstacle_range_);
  this->get_parameter("width_regions", width_steps_);
  this->get_parameter("height_regions", height_steps_);
  this->get_parameter("max_distance_for_visualization", max_distance_);
  this->get_parameter("obstacle_state_topic", obstacle_state_topic_);

  // Timer for initializing the node after the construction is done to avoid bad weak_ptr error
  timer_ = this->create_wall_timer(std::chrono::milliseconds(5), std::bind(&DepthObstacleDetect::init, this));
}

void DepthObstacleDetect::init()
{
  // Disabling timer based callback to avoid re-initializations
  timer_->cancel();
  image_transport::ImageTransport it(this->shared_from_this());
  init_distance_conversion_ = true;
  sub_depth_ = it.subscribe(depth_topic_, 1, &DepthObstacleDetect::imageCallback, this);
  sub_info_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
      camera_info_topic_, 10, std::bind(&DepthObstacleDetect::infoCallback, this, std::placeholders::_1));
  pub_detect_ = it.advertise(detect_topic_, 1);
  pub_obstacle_ = create_publisher<depth_obstacle_detect_ros_msg::msg::ObstacleStampedArray>(obstacle_state_topic_, 1);

  msg_header_detect_.frame_id = cam_id_;
  msg_header_detect_.stamp = this->get_clock()->now();

  obstacle_range_threshold_ = obstacle_range_ * 1000.0;
  msg_header_obstacle_.frame_id = cam_id_;
  msg_header_obstacle_.stamp = this->get_clock()->now();
  msg_obstacle_.header = msg_header_obstacle_;

  RCLCPP_INFO(this->get_logger(), "##### Image Width Regions  = %d, Image Height Regions  = %d. #####", width_steps_,
              height_steps_);
  RCLCPP_INFO(this->get_logger(), "##### Obstacle Range Limit = %fm (or %fmm). #####", obstacle_range_,
              obstacle_range_threshold_);
}

void DepthObstacleDetect::imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr& msg)
{
  try
  {
    // Convert Image message to cv::Mat
    if (msg->encoding == "mono16" || msg->encoding == "16UC1")
    {
      detect_image_ = cv_bridge::toCvCopy(msg, "16UC1")->image;
      is32FC1_ = false;
      if (verbose_)
        RCLCPP_INFO(this->get_logger(), "Detected 16UC1 Image");
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
        RCLCPP_INFO(this->get_logger(), "Detected 32FC1 Image");
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
    cv::inRange(detect_image_, 0, 0, fov_border_mask_);     // Create the border mask based on 0 value pixels/
    detect_image_.setTo(image_maxValue_, fov_border_mask_);  // Use the mask to set all of the border pixel values to the
                                                          // max image pixel value.

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
          RCLCPP_INFO(this->get_logger(), "%f", minValue_);
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
            RCLCPP_INFO(this->get_logger(), "Obstacle Detected in Region (%d, %d), Range = %fm.", i, j,
                        minValue_ / 1000.0);
        }
      }
    }

    // Publish output Image and obstacle state message.
    msg_header_detect_.stamp = this->get_clock()->now();
    sensor_msgs::msg::Image::ConstSharedPtr msg_detect =
        cv_bridge::CvImage(msg_header_detect_, "rgb8", detect_image_out_).toImageMsg();
    pub_detect_.publish(msg_detect);
    msg_obstacle_.header.stamp = msg_header_detect_.stamp;
    pub_obstacle_->publish(msg_obstacle_);
  }
  catch (cv_bridge::Exception& e)
  {
    RCLCPP_ERROR(this->get_logger(), "Error in Callback OpenCV processing.");
  }
}

void DepthObstacleDetect::infoCallback(const sensor_msgs::msg::CameraInfo& msg)
{
  cam_info_detect_ = msg;
  // Do nothing else - may be used to add future functionality.
}