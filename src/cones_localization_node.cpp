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

using std::placeholders::_1;
using std::placeholders::_2;
using std::placeholders::_3;

ConesLocalizationNode::ConesLocalizationNode(const rclcpp::NodeOptions & options)
:  Node("cones_localization", options)
{
  sub1_.subscribe(this, "/output_bboxes", rmw_qos_profile_sensor_data);
  sub2_.subscribe(this, "/output_image", rmw_qos_profile_sensor_data);
  sub3_.subscribe(this, "/sensing/lidar/scan", rmw_qos_profile_sensor_data);
  
  my_sync_ = std::make_shared<approximate_synchronizer>(approximate_policy(10), sub1_, sub2_, sub3_);
  my_sync_->getPolicy()->setMaxIntervalDuration(rclcpp::Duration(20,0));
  my_sync_->registerCallback(std::bind(&ConesLocalizationNode::callbackSync, this, _1, _2, _3));

  cones_localization_ = std::make_unique<cones_localization::ConesLocalization>();
  param_name_ = this->declare_parameter("param_name", 456);
  cones_localization_->foo(param_name_);

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

void ConesLocalizationNode::callbackSync(const cones_interfaces::msg::Cones::ConstSharedPtr &bboxes_msg,
                                          const sensor_msgs::msg::Image::ConstSharedPtr &image_msg,
                                          const sensor_msgs::msg::LaserScan::ConstSharedPtr &lidar_msg) const {
  RCLCPP_INFO(this->get_logger(), "Received synchronized messages:");
  RCLCPP_INFO(this->get_logger(), "Time difference between lidar and bbox: %f", abs((bboxes_msg->header.stamp.sec + bboxes_msg->header.stamp.nanosec * 1e-9) - (lidar_msg->header.stamp.sec + lidar_msg->header.stamp.nanosec * 1e-9)));
  RCLCPP_INFO(this->get_logger(), "Time difference between lidar and image: %f", abs((image_msg->header.stamp.sec + image_msg->header.stamp.nanosec * 1e-9) - (lidar_msg->header.stamp.sec + lidar_msg->header.stamp.nanosec * 1e-9)));
  // RCLCPP_INFO(this->get_logger(), "  topic3: %u", lidar_msg->header.stamp.sec);

  cones_localization_->lidarProcessing(lidar_msg, fx_, cx_, camera_fov_horizontal_, image_height_);

  cones_localization_->bboxesProcessing(bboxes_msg);

  cones_localization_->imageProcessing(image_msg);

  // map_msg_local_ = cones_localization_->localizationProcessing(localization_msg, map_msg_);

  if (map_msg_local_ != nullptr){
  map_pub_->publish(*map_msg_local_); 
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
