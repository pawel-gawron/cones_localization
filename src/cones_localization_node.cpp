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

template <typename T>
T findClosestMessage(std::queue<T> &queue, float target_time) {
    T closest_msg; 
    float min_diff = std::numeric_limits<float>::max();

    int i = 0;

    std::queue<T> queue_copy = queue;

    while (!queue_copy.empty()) {
        auto msg = queue_copy.front();
        float msg_time = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;
        float diff = std::abs(msg_time - target_time);
        i++;

        if (diff < min_diff) {
            min_diff = diff;
            closest_msg = msg;
        }

        queue_copy.pop();
    }
    // std::cout << "Minimum difference: " << min_diff << ", " << i << std::endl;
    return closest_msg;
}

namespace cones_localization
{

auto custom_qos = rclcpp::QoS(rclcpp::KeepLast(1), rmw_qos_profile_sensor_data);

ConesLocalizationNode::ConesLocalizationNode(const rclcpp::NodeOptions & options)
:  Node("cones_localization", options)
{
  cones_localization_ = std::make_unique<cones_localization::ConesLocalization>();
  param_name_ = this->declare_parameter("param_name", 456);
  cones_localization_->foo(param_name_);

  timer_ = this->create_wall_timer(std::chrono::milliseconds(10), std::bind(&ConesLocalizationNode::timerCallback, this));

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

  localization_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
  "/localization/cartographer/pose",
  custom_qos,
  std::bind(&ConesLocalizationNode::localizationCallback, this, 
  std::placeholders::_1));

  camera_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
  "/sensing/camera/camera_info",
  custom_qos,
  std::bind(&ConesLocalizationNode::cameraInfoCallback, this, 
  std::placeholders::_1));

  map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
  "/map",
  custom_qos,
  std::bind(&ConesLocalizationNode::mapCallback, this, 
  std::placeholders::_1));

  map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("output_map", custom_qos);
}

void ConesLocalizationNode::timerCallback()
{
  if (!lidar_queue_.empty() && !bboxes_queue_.empty() && !image_queue_.empty() && !localization_queue_.empty())
  {
    float time_lidar;
    {
      std::lock_guard<std::recursive_mutex> lidar_lock(lidar_queue_mutex_);
      auto lidar_msg = lidar_queue_.back();
      time_lidar = lidar_msg->header.stamp.sec + lidar_msg->header.stamp.nanosec * 1e-9;
      cones_localization_->lidarProcessing(lidar_msg, fx_, cx_, camera_fov_horizontal_, image_height_);
    }

    {
      std::lock_guard<std::recursive_mutex> bboxes_lock(bboxes_queue_mutex_);
      auto bboxes_msg = findClosestMessage(bboxes_queue_, time_lidar);
      cones_localization_->bboxesProcessing(bboxes_msg);
    }

    {
      std::lock_guard<std::recursive_mutex> image_lock(image_queue_mutex_);
      auto image_msg = findClosestMessage(image_queue_, time_lidar);
      cones_localization_->imageProcessing(image_msg);
    }

    {
      std::lock_guard<std::recursive_mutex> loc_lock(localization_queue_mutex_);
      auto localization_msg = findClosestMessage(localization_queue_, time_lidar);
      map_msg_local_ = cones_localization_->localizationProcessing(localization_msg, map_msg_);

      // if (map_msg_local_ != nullptr){
      // // map_pub_->publish(*map_msg_local_); 
      // }
    }
  }
}

void ConesLocalizationNode::cameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
{
    if (!executed_camera_info_)
    {
      fx_ = msg->k[0];
      cx_ = msg->k[2];
      fy_ = msg->k[4];
      cy_ = msg->k[5];

      camera_matrix_ = cv::Mat(3, 3, CV_64F, msg->k.data());
      distortion_coefficients_ = cv::Mat(1, 5, CV_64F, msg->d.data());
      // float cy = msg->k[5]; 
      image_width_ = msg->width;
      image_height_ = msg->height;

      max_x_camera_ = (image_width_ - 1 - cx_) / fx_;
      min_x_camera_ = -cx_ / fx_;
      angle_max_ = std::atan(max_x_camera_ / fx_);

      camera_fov_horizontal_ = 2 * std::atan(image_width_ / (2 * fx_));
      camera_fov_vertical_ = 2 * std::atan(image_height_ / (2 * fy_));

      // max_y_camera_ = (image_height_ - 1 - cy) / fy;
      // min_y_camera_ = -cy / fy;

      angle_min_ = -angle_max_;

      max_y_camera_ = image_height_ - 1;  // Maksymalna współrzędna Y piksela
      min_y_camera_ = -0.1;

      executed_camera_info_ = true;
    }
}

void ConesLocalizationNode::bboxesCallback(const cones_interfaces::msg::Cones::SharedPtr msg)
{
  std::lock_guard<std::recursive_mutex> bboxes_lock(bboxes_queue_mutex_);
  if (bboxes_queue_.size() >= 10) {
    bboxes_queue_.pop();
  }
  bboxes_queue_.push(msg);
}

void ConesLocalizationNode::imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
{
  std::lock_guard<std::recursive_mutex> image_lock(image_queue_mutex_);
  if (image_queue_.size() >= 10) { 
    image_queue_.pop();
  }
  image_queue_.push(msg);
}

void ConesLocalizationNode::lidarCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
  std::lock_guard<std::recursive_mutex> lidar_lock(lidar_queue_mutex_);
  if (!lidar_queue_.empty()) {
    lidar_queue_.pop();
  }
  lidar_queue_.push(msg);
}

void ConesLocalizationNode::localizationCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  std::lock_guard<std::recursive_mutex> loc_lock(localization_queue_mutex_);
  if (localization_queue_.size() >= 10) {
    localization_queue_.pop();
  }
  localization_queue_.push(msg);
}

void ConesLocalizationNode::mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
  map_msg_ = msg;
}

}  // namespace cones_localization

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(cones_localization::ConesLocalizationNode)
