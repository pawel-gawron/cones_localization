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

#include <chrono>
#include <inttypes.h>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include "rclcpp/time.hpp"
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <queue>
#include <cmath>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>


#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/camera_info.hpp>

#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>

#include "cones_localization/cones_localization.hpp"
#include "cones_interfaces/msg/bounding_box.hpp" 
#include "cones_interfaces/msg/cones.hpp"

#include <rmw/qos_profiles.h>

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

  std::queue<sensor_msgs::msg::LaserScan::SharedPtr> lidar_queue_;
  std::queue<cones_interfaces::msg::Cones::SharedPtr> bboxes_queue_;
  std::queue<sensor_msgs::msg::Image::SharedPtr> image_queue_;
  std::queue<geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr> localization_queue_;

  rclcpp::TimerBase::SharedPtr timer_;

  std::recursive_mutex lidar_queue_mutex_;
  std::recursive_mutex bboxes_queue_mutex_;
  std::recursive_mutex image_queue_mutex_;
  std::recursive_mutex localization_queue_mutex_;

  typedef message_filters::sync_policies::ApproximateTime<cones_interfaces::msg::Cones,
                                                    sensor_msgs::msg::Image,
                                                    sensor_msgs::msg::LaserScan> approx_policy;

  void timerCallback();

  void bboxesCallback(const cones_interfaces::msg::Cones::SharedPtr msg);
  void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg);
  void lidarCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
  void localizationCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
  void cameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg);

  void approxCallback(const cones_interfaces::msg::Cones::SharedPtr cones_msg,
                                            const sensor_msgs::msg::Image::SharedPtr image_msg,
                                            const sensor_msgs::msg::LaserScan::SharedPtr lidar_msg);

  // void callbackFunction(const std::shared_ptr<const cones_interfaces::msg::Cones>& cones_msg,
  //                     const std::shared_ptr<const sensor_msgs::msg::Image>& image_msg,
  //                     const std::shared_ptr<const sensor_msgs::msg::LaserScan>& lidar_msg);

  rclcpp::Subscription<cones_interfaces::msg::Cones>::SharedPtr bboxes_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr localization_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_sub_;

  rmw_qos_profile_t custom_qos_profile = rmw_qos_profile_default;
};
}  // namespace cones_localization

#endif  // CONES_LOCALIZATION__CONES_LOCALIZATION_NODE_HPP_
