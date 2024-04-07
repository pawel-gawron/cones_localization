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

#ifndef CONES_LOCALIZATION__CONES_LOCALIZATION_NODE_HPP_
#define CONES_LOCALIZATION__CONES_LOCALIZATION_NODE_HPP_

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/camera_info.hpp>

#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>

#include "cones_localization/cones_localization.hpp"
#include "cones_interfaces/msg/bounding_box.hpp" 
#include "cones_interfaces/msg/cones.hpp" 

namespace cones_localization
{
using ConesLocalizationPtr = std::unique_ptr<cones_localization::ConesLocalization>;

class CONES_LOCALIZATION_PUBLIC ConesLocalizationNode : public rclcpp::Node
{
public:
  explicit ConesLocalizationNode(const rclcpp::NodeOptions & options);

private:
  ConesLocalizationPtr cones_localization_{nullptr};
  int64_t param_name_{123};
  int image_width_;
  int image_height_;
  bool executed_ = false;

  float max_x_camera_;
  float max_y_camera_;
  float min_x_camera_;
  float min_y_camera_;
  float camera_fov_horizontal_;
  float camera_fov_vertical_;
  float fx_;
  float cx_;

  cv::Mat camera_matrix_; // Macierz kamery
  cv::Mat distortion_coefficients_;

  float angle_max_;
  float angle_min_;

  std::vector<std::tuple<float, float, float>> lidar_points_;
  std::vector<float> bboxes_points_;
  std::unique_ptr<cones_interfaces::msg::Cones> cones_ = std::make_unique<cones_interfaces::msg::Cones>();
  float max_range_;

  void bboxesCallback(const cones_interfaces::msg::Cones::SharedPtr msg);
  void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg);
  void lidarCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
  void localizationCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
  void cameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg);
  rclcpp::Subscription<cones_interfaces::msg::Cones>::SharedPtr bboxes_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr localization_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_sub_;
};
}  // namespace cones_localization

#endif  // CONES_LOCALIZATION__CONES_LOCALIZATION_NODE_HPP_
