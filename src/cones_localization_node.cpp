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

  custom_qos_profile.depth = 100;
  custom_qos_profile.reliability = rmw_qos_reliability_policy_t::RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;
  custom_qos_profile.history = rmw_qos_history_policy_t::RMW_QOS_POLICY_HISTORY_KEEP_LAST;
  custom_qos_profile.durability = rmw_qos_durability_policy_t::RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL;

  message_filters::Subscriber<cones_interfaces::msg::Cones> bboxes_sub_exact_(this, "/output_bboxes", custom_qos_profile);
  message_filters::Subscriber<sensor_msgs::msg::Image> image_sub_exact_(this, "/output_image", custom_qos_profile);
  message_filters::Subscriber<sensor_msgs::msg::LaserScan> lidar_sub_exact_(this, "/sensing/lidar/scan", custom_qos_profile);

  message_filters::Synchronizer<approx_policy>syncApprox(approx_policy(10), bboxes_sub_exact_, image_sub_exact_, lidar_sub_exact_);


  cones_localization_ = std::make_unique<cones_localization::ConesLocalization>();
  param_name_ = this->declare_parameter("param_name", 456);
  cones_localization_->foo(param_name_);

  syncApprox.registerCallback(std::bind(&ConesLocalizationNode::approxCallback, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3,
  std::placeholders::_4, std::placeholders::_5, std::placeholders::_6,
  std::placeholders::_7, std::placeholders::_8, std::placeholders::_9));

  RCLCPP_INFO(this->get_logger(), "ConesLocalizationNode initialized successfully.");

  // timer_ = this->create_wall_timer(std::chrono::milliseconds(10), std::bind(&ConesLocalizationNode::timerCallback, this));

  // bboxes_sub_ = this->create_subscription<cones_interfaces::msg::Cones>(
  // "/output_bboxes",
  // custom_qos,
  // std::bind(&ConesLocalizationNode::bboxesCallback, this, 
  // std::placeholders::_1));

  // image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
  // "/output_image",
  // custom_qos,
  // std::bind(&ConesLocalizationNode::imageCallback, this, 
  // std::placeholders::_1));

  // lidar_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
  // "/sensing/lidar/scan",
  // custom_qos,
  // std::bind(&ConesLocalizationNode::lidarCallback, this, 
  // std::placeholders::_1));

  // localization_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
  // "/localization/pose_twist_fusion_filter/pose_with_covariance",
  // custom_qos,
  // std::bind(&ConesLocalizationNode::localizationCallback, this, 
  // std::placeholders::_1));

  // camera_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
  // "/sensing/camera/camera_info",
  // custom_qos,
  // std::bind(&ConesLocalizationNode::cameraInfoCallback, this, 
  // std::placeholders::_1));
}

void ConesLocalizationNode::approxCallback(const cones_interfaces::msg::Cones::SharedPtr cones_msg,
                                            const sensor_msgs::msg::Image::SharedPtr image_msg,
                                            const sensor_msgs::msg::LaserScan::SharedPtr lidar_msg)
{
  std::cout << "Callback" << std::endl;
  std::cout << "Cones: " << cones_msg->header.stamp.sec << std::endl;
  std::cout << "Image: " << image_msg->header.stamp.sec << std::endl;
  std::cout << "Lidar: " << lidar_msg->header.stamp.sec << std::endl;
}

// void ConesLocalizationNode::timerCallback()
// {
//   std::lock_guard<std::recursive_mutex> lidar_lock(lidar_queue_mutex_);
//   std::lock_guard<std::recursive_mutex> bboxes_lock(bboxes_queue_mutex_);
//   std::lock_guard<std::recursive_mutex> image_lock(image_queue_mutex_);
//   std::lock_guard<std::recursive_mutex> loc_lock(localization_queue_mutex_);

//   if (!lidar_queue_.empty() && !bboxes_queue_.empty() && !image_queue_.empty() && !localization_queue_.empty())
//   {
//     auto lidar_msg = lidar_queue_.back();
//     lidar_queue_.pop();
//     cones_localization_->lidarProcessing(lidar_msg, fx_, cx_, camera_fov_horizontal_, image_height_);

//     auto bboxes_msg = bboxes_queue_.back();
//     bboxes_queue_.pop();
//     cones_localization_->bboxesProcessing(bboxes_msg);

//     auto image_msg = image_queue_.back();
//     image_queue_.pop();
//     cones_localization_->imageProcessing(image_msg);

//     auto localization_msg = localization_queue_.back();
//     localization_queue_.pop();
//     cones_localization_->localizationProcessing(localization_msg);
//   }
// }

// void ConesLocalizationNode::cameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
// {
//     if (!executed_)
//     {
//       fx_ = msg->k[0];
//       cx_ = msg->k[2];
//       float fy = msg->k[4];
//       camera_matrix_ = cv::Mat(3, 3, CV_64F, msg->k.data());
//       distortion_coefficients_ = cv::Mat(1, 5, CV_64F, msg->d.data());
//       // float cy = msg->k[5]; 
//       image_width_ = msg->width;
//       image_height_ = msg->height;

//       max_x_camera_ = (image_width_ - 1 - cx_) / fx_;
//       min_x_camera_ = -cx_ / fx_;
//       angle_max_ = std::atan(max_x_camera_ / fx_);

//       camera_fov_horizontal_ = 2 * std::atan(image_width_ / (2 * fx_));
//       camera_fov_vertical_ = 2 * std::atan(image_height_ / (2 * fy));

//       // max_y_camera_ = (image_height_ - 1 - cy) / fy;
//       // min_y_camera_ = -cy / fy;

//       angle_min_ = -angle_max_;

//       max_y_camera_ = image_height_ - 1;  // Maksymalna współrzędna Y piksela
//       min_y_camera_ = -0.1;

//       executed_ = true;
//     }
// }

// void ConesLocalizationNode::bboxesCallback(const cones_interfaces::msg::Cones::SharedPtr msg)
// {
//   std::lock_guard<std::recursive_mutex> bboxes_lock(bboxes_queue_mutex_);
//   if (!bboxes_queue_.empty()) {
//       bboxes_queue_.pop();
//     }
//     bboxes_queue_.push(msg);
// }

// void ConesLocalizationNode::imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
// {
//   std::lock_guard<std::recursive_mutex> image_lock(image_queue_mutex_);
//   if (!image_queue_.empty()) {
//     image_queue_.pop();
//   }
//   image_queue_.push(msg);
// }

// void ConesLocalizationNode::lidarCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
// {
//   std::lock_guard<std::recursive_mutex> lidar_lock(lidar_queue_mutex_);
//   if (!lidar_queue_.empty()) {
//     lidar_queue_.pop();
//   }
//   lidar_queue_.push(msg);
// }

// void ConesLocalizationNode::localizationCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
// {
//   std::lock_guard<std::recursive_mutex> loc_lock(localization_queue_mutex_);
//   if (!localization_queue_.empty()) {
//     localization_queue_.pop();
//   }
//   localization_queue_.push(msg);
// }

}  // namespace cones_localization

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(cones_localization::ConesLocalizationNode)
