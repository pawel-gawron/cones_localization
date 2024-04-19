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
#include "message_filters/time_synchronizer.h"
#include <message_filters/sync_policies/approximate_time.h>


#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose.hpp>
// #include <geometry_msgs/msg/transform_stamped.hpp>

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

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
  bool executed_camera_info_ = false;
  bool executed_map_ = false;

  float max_x_camera_;
  float max_y_camera_;
  float min_x_camera_;
  float min_y_camera_;
  float camera_fov_horizontal_;
  float camera_fov_vertical_;
  float fx_;
  float cx_;
  float fy_;
  float cy_;

  int cones_number_map_;

  nav_msgs::msg::OccupancyGrid::SharedPtr map_msg_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  cv::Mat camera_matrix_; // Macierz kamery
  cv::Mat distortion_coefficients_;

  float angle_max_;
  float angle_min_;

  geometry_msgs::msg::TransformStamped get_transform(const std::string& from, const std::string& to) const;

  typedef message_filters::sync_policies::ApproximateTime<cones_interfaces::msg::Cones,
                                                    sensor_msgs::msg::Image,
                                                    sensor_msgs::msg::LaserScan,
                                                    geometry_msgs::msg::PoseStamped> approximate_policy;
  typedef message_filters::Synchronizer<approximate_policy> approximate_synchronizer;

  void cameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg);
  void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
  void callbackSync(const cones_interfaces::msg::Cones::ConstSharedPtr &bboxes_msg,
                    const sensor_msgs::msg::Image::ConstSharedPtr &image_msg,
                    const sensor_msgs::msg::LaserScan::ConstSharedPtr &lidar_msg,
                    const geometry_msgs::msg::PoseStamped::ConstSharedPtr &loc_msg) const;

  message_filters::Subscriber<cones_interfaces::msg::Cones> bboxes_sub_;
  message_filters::Subscriber<sensor_msgs::msg::Image> image_sub_;
  message_filters::Subscriber<sensor_msgs::msg::LaserScan> lidar_sub_;
  message_filters::Subscriber<geometry_msgs::msg::PoseStamped> localization_sub_;

  std::shared_ptr<approximate_synchronizer> my_sync_;

  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;

  rmw_qos_profile_t custom_qos_profile = rmw_qos_profile_default;
};
}  // namespace cones_localization

#endif  // CONES_LOCALIZATION__CONES_LOCALIZATION_NODE_HPP_
