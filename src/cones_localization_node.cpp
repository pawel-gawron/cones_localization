// Copyright 2024 PawelGawron
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "cones_localization/cones_localization_node.hpp"

namespace cones_localization
{
auto custom_qos = rclcpp::QoS(rclcpp::KeepLast(1), rmw_qos_profile_sensor_data);

ConesLocalizationNode::ConesLocalizationNode(const rclcpp::NodeOptions & options)
:  Node("cones_localization", options)
{
  cones_localization_ = std::make_unique<cones_localization::ConesLocalization>();
  param_name_ = this->declare_parameter("param_name", 456);
  cones_localization_->foo(param_name_);

  timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&ConesLocalizationNode::timerCallback, this));

  bboxes_sub_ = this->create_subscription<cones_interfaces::msg::Cones>(
  "/output_bboxes",
  custom_qos,
  std::bind(&ConesLocalizationNode::bboxesCallback, this, 
  std::placeholders::_1));

  image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
  "/output_image",
  custom_qos,
  std::bind(&ConesLocalizationNode::imageCallback, this, 
  std::placeholders::_1));

  lidar_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
  "/sensing/lidar/scan",
  custom_qos,
  std::bind(&ConesLocalizationNode::lidarCallback, this, 
  std::placeholders::_1));

  localization_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
  "/localization/pose_twist_fusion_filter/pose_with_covariance",
  custom_qos,
  std::bind(&ConesLocalizationNode::localizationCallback, this, 
  std::placeholders::_1));

  camera_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
  "/sensing/camera/camera_info",
  custom_qos,
  std::bind(&ConesLocalizationNode::cameraInfoCallback, this, 
  std::placeholders::_1));
}

void ConesLocalizationNode::timerCallback()
{
  std::cout << "Timer callback" << std::endl;
    if (!lidar_queue_.empty() && !bboxes_queue_.empty() && !image_queue_.empty() && !localization_queue_.empty())
    {
        auto lidar_msg = lidar_queue_.front();
        lidar_queue_.pop();
        lidar_points_test_ = cones_localization_->lidarProcessing(lidar_msg, fx_, cx_, camera_fov_horizontal_, image_height_);

        auto bboxes_msg = bboxes_queue_.front();
        bboxes_queue_.pop();
        cones_test_ = cones_localization_->bboxesProcessing(bboxes_msg);

        auto image_msg = image_queue_.front();
        image_queue_.pop();
        cones_localization_->imageProcessing(image_msg, lidar_points_test_, lidar_points_test_, cones_test_);

        auto localization_msg = localization_queue_.front();
        localization_queue_.pop();
        cones_localization_->localizationProcessing(localization_msg);
    }
}

void ConesLocalizationNode::cameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
{
    if (!executed_)
    {
      fx_ = msg->k[0];
      cx_ = msg->k[2];
      float fy = msg->k[4];
      camera_matrix_ = cv::Mat(3, 3, CV_64F, msg->k.data());
      distortion_coefficients_ = cv::Mat(1, 5, CV_64F, msg->d.data());
      // float cy = msg->k[5]; 
      image_width_ = msg->width;
      image_height_ = msg->height;

      max_x_camera_ = (image_width_ - 1 - cx_) / fx_;
      min_x_camera_ = -cx_ / fx_;
      angle_max_ = std::atan(max_x_camera_ / fx_);

      camera_fov_horizontal_ = 2 * std::atan(image_width_ / (2 * fx_));
      camera_fov_vertical_ = 2 * std::atan(image_height_ / (2 * fy));

      // max_y_camera_ = (image_height_ - 1 - cy) / fy;
      // min_y_camera_ = -cy / fy;

      angle_min_ = -angle_max_;

      max_y_camera_ = image_height_ - 1;  // Maksymalna współrzędna Y piksela
      min_y_camera_ = -0.1;

      executed_ = true;
    }
}

void ConesLocalizationNode::bboxesCallback(const cones_interfaces::msg::Cones::SharedPtr msg)
{
  cones_->header = msg->header;
  cones_->bboxes = msg->bboxes;

  if (!bboxes_queue_.empty()) {
      bboxes_queue_.pop();
    }
    bboxes_queue_.push(msg);

  // for (const auto& bbox : cones->bboxes)
  // {
  //   std::cout << "Bbox: " << bbox.x1 << " " << bbox.x2 << " " << bbox.y1 << " " << bbox.y2 << std::endl;
  // }
}

void ConesLocalizationNode::imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
{
  if (!image_queue_.empty()) {
    image_queue_.pop();
  }
  image_queue_.push(msg);
  // RCLCPP_INFO(this->get_logger(), "Received image message");
  cv::Mat frame_cv;
  frame_cv = cv_bridge::toCvCopy(msg, "bgr8")->image;
  
  for (const auto& bbox : cones_->bboxes)
  {
    bboxes_points_.clear();
    for (const auto& point : lidar_points_)
    {
      if(std::get<0>(point) >= bbox.x1 && std::get<0>(point) <= bbox.x2)
      {
        cv::circle(frame_cv, cv::Point(std::get<0>(point), std::get<1>(point)), 2, cv::Scalar(0, 0, 255), -1);
        bboxes_points_.push_back(std::get<2>(point));
      }
    }
    if (!bboxes_points_.empty())
    {
      // Sort the points in ascending order
      std::sort(bboxes_points_.begin(), bboxes_points_.end());

      // Calculate the median
      float median;
      if (bboxes_points_.size() % 2 == 0)
      {
        median = (bboxes_points_[bboxes_points_.size() / 2 - 1] + bboxes_points_[bboxes_points_.size() / 2]) / 2.0;
      }
      else
      {
        median = bboxes_points_[bboxes_points_.size() / 2];
      }

      float rounded_median = std::round(median * 100) / 100;
      std::stringstream ss;
      ss << std::fixed << std::setprecision(2) << rounded_median;
      cv::putText(frame_cv, ss.str(), cv::Point(bbox.x2, bbox.y1), cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0, 0, 255), 2);
    }
  }

  cv::imshow("Cones from localization", frame_cv);
  cv::waitKey(1);
}

void ConesLocalizationNode::lidarCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
  if (!lidar_queue_.empty()) {
    lidar_queue_.pop();
  }
  lidar_queue_.push(msg);

  auto scan_msg = std::make_unique<sensor_msgs::msg::LaserScan>();
  scan_msg->header = msg->header;

  scan_msg->angle_min = msg->angle_min;
  scan_msg->angle_max = msg->angle_max;
  scan_msg->angle_increment = msg->angle_increment;
  scan_msg->time_increment = msg->time_increment;
  scan_msg->scan_time = msg->scan_time;
  scan_msg->range_min = msg->range_min;
  scan_msg->range_max = msg->range_max;

  // determine amount of rays to create
  uint32_t ranges_size = std::ceil(
    (scan_msg->angle_max - scan_msg->angle_min) / scan_msg->angle_increment);

  // std::cout << "ranges_size: " << ranges_size << std::endl;


  lidar_points_.clear();

  float min_angle = msg->angle_min;
  float max_angle = msg->angle_max;
  float angle_increment = msg->angle_increment;

  // float lidar_fov = max_angle - min_angle; 

  std::vector<float> ranges = msg->ranges;

  const float camera_offset = 0.1;
  max_range_ = msg->range_max;
  int y_pixel;
  int x_pixel;
  float x_camera;
  float normalized_angle;
  float alpha;
  float angle;
  float distance;

  for (uint32_t i = 0; i < ranges_size; ++i)
  { 
    angle = max_angle - i * angle_increment;    
    distance = ranges[i];

    // Check if point is within camera FOV 
    if (angle >= min_angle && angle <= max_angle && angle <= camera_fov_horizontal_/2 && angle >= -camera_fov_horizontal_/2)
    {
        // Normalize angle to be between 0 and 1 relative to camera FOV
        normalized_angle = ((angle+(camera_fov_horizontal_/2)) / (camera_fov_horizontal_));

        x_camera = tan(normalized_angle * camera_fov_horizontal_ - camera_fov_horizontal_ / 2);
        x_pixel = static_cast<int>(fx_ * x_camera + cx_);

        if (distance >= max_range_) {
            y_pixel = image_height_ / 2;
            continue;
        }

        alpha = atan(distance / camera_offset);

        y_pixel = static_cast<int>((alpha/1.5708) * image_height_/2);

        lidar_points_.emplace_back(x_pixel, y_pixel, distance);
    }  
  }
}

void ConesLocalizationNode::localizationCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
  if (!localization_queue_.empty()) {
    localization_queue_.pop();
  }
  localization_queue_.push(msg);
  RCLCPP_INFO(this->get_logger(), "Received localization message");
  geometry_msgs::msg::PoseWithCovarianceStamped imu;
  imu.header = msg->header;
  // RCLCPP_INFO(this->get_logger(), imu.header);
  std::cout << "Message data: " << msg->header.frame_id << std::endl;


}

}  // namespace cones_localization

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(cones_localization::ConesLocalizationNode)
