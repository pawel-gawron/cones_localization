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

geometry_msgs::msg::Pose transform_pose(const geometry_msgs::msg::Pose& pose, const geometry_msgs::msg::TransformStamped& transform)
{
  geometry_msgs::msg::PoseStamped transformed_pose;
  geometry_msgs::msg::PoseStamped orig_pose;
  orig_pose.pose = pose;
  tf2::doTransform(orig_pose, transformed_pose, transform);

  return transformed_pose.pose;
}

namespace cones_localization
{

auto custom_qos = rclcpp::QoS(rclcpp::KeepLast(1), rmw_qos_profile_sensor_data);

using std::placeholders::_1;
using std::placeholders::_2;
using std::placeholders::_3;
using std::placeholders::_4;

ConesLocalizationNode::ConesLocalizationNode(const rclcpp::NodeOptions & options)
:  Node("cones_localization", options)
{
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  cones_number_map_ = declare_parameter<int>("cones_number_map");

  bboxes_sub_.subscribe(this, "/output_bboxes", rmw_qos_profile_sensor_data);
  image_sub_.subscribe(this, "/output_image", rmw_qos_profile_sensor_data);
  lidar_sub_.subscribe(this, "/sensing/lidar/scan", rmw_qos_profile_sensor_data);
  localization_sub_.subscribe(this, "/localization/cartographer/pose", rmw_qos_profile_sensor_data);
  
  my_sync_ = std::make_shared<approximate_synchronizer>(approximate_policy(5),
                                                        bboxes_sub_, image_sub_, lidar_sub_, localization_sub_);
  my_sync_->getPolicy()->setMaxIntervalDuration(rclcpp::Duration(10,0));
  my_sync_->registerCallback(std::bind(&ConesLocalizationNode::callbackSync, this, _1, _2, _3, _4));

  cones_localization_ = std::make_unique<cones_localization::ConesLocalization>();
  param_name_ = this->declare_parameter("param_name", 456);
  cones_localization_->foo(param_name_);

  cones_localization_->setConfig(cones_number_map_);

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
                                          const sensor_msgs::msg::LaserScan::ConstSharedPtr &lidar_msg,
                                          const geometry_msgs::msg::PoseStamped::ConstSharedPtr &loc_msg) {
  // RCLCPP_INFO(this->get_logger(), "Received synchronized messages:");
  // RCLCPP_INFO(this->get_logger(), "Time difference between lidar and bbox: %f", abs((bboxes_msg->header.stamp.sec + bboxes_msg->header.stamp.nanosec * 1e-9) - (lidar_msg->header.stamp.sec + lidar_msg->header.stamp.nanosec * 1e-9)));
  // RCLCPP_INFO(this->get_logger(), "Time difference between lidar and image: %f", abs((image_msg->header.stamp.sec + image_msg->header.stamp.nanosec * 1e-9) - (lidar_msg->header.stamp.sec + lidar_msg->header.stamp.nanosec * 1e-9)));
  // RCLCPP_INFO(this->get_logger(), "Time difference between lidar and image: %f", abs((loc_msg->header.stamp.sec + loc_msg->header.stamp.nanosec * 1e-9) - (lidar_msg->header.stamp.sec + lidar_msg->header.stamp.nanosec * 1e-9)));

  if (map_msg_ && loc_msg)
  {
    float current_time = lidar_msg->header.stamp.sec + lidar_msg->header.stamp.nanosec * 1e-9;
    float time_difference = current_time - previous_time_;

    cones_localization_->lidarProcessing(lidar_msg, fx_, cx_, camera_fov_horizontal_, image_height_, car_yaw_velocity_);
    cones_localization_->bboxesProcessing(bboxes_msg);
    cones_localization_->imageProcessing(image_msg);

    double car_yaw;
    tf2::Quaternion car_orientation(loc_msg->pose.orientation.x,
                                    loc_msg->pose.orientation.y,
                                    loc_msg->pose.orientation.z,
                                    loc_msg->pose.orientation.w);
    tf2::Matrix3x3 matrix(car_orientation);
    double roll, pitch;
    matrix.getRPY(roll, pitch, car_yaw);

    float delta_yaw = car_yaw - previous_car_yaw_; 
    car_yaw_velocity_ = delta_yaw / time_difference;

    previous_car_yaw_ = car_yaw;
    previous_time_ = current_time;

    const auto current_pose_in_costmap_frame = transform_pose(
    loc_msg->pose,
    get_transform(map_msg_->header.frame_id,
                  loc_msg->header.frame_id));

    // RCLCPP_INFO(this->get_logger(), "Current pose in costmap frame: %f %f",
    //             current_pose_in_costmap_frame.position.x,
    //             current_pose_in_costmap_frame.position.y);

    nav_msgs::msg::OccupancyGrid::ConstSharedPtr map_msg_local = cones_localization_->localizationProcessing(current_pose_in_costmap_frame, map_msg_, car_yaw);

    map_pub_->publish(*map_msg_local);
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

void ConesLocalizationNode::mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
  if(!executed_map_)
  {
    map_msg_ = msg;
    executed_map_ = true;
  }
}

geometry_msgs::msg::TransformStamped ConesLocalizationNode::get_transform(
  const std::string& from,
  const std::string& to)
{
  geometry_msgs::msg::TransformStamped tf;
  try {
    tf =
      tf_buffer_->lookupTransform(from, to, rclcpp::Time(0), rclcpp::Duration::from_seconds(1.0));
  } catch (const tf2::TransformException & ex) {
    RCLCPP_ERROR(this->get_logger(), "%s", ex.what());
  }
  return tf;
}

}  // namespace cones_localization

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(cones_localization::ConesLocalizationNode)
