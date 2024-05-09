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

#ifndef CONES_LOCALIZATION__CONES_LOCALIZATION_HPP_
#define CONES_LOCALIZATION__CONES_LOCALIZATION_HPP_

#include <rclcpp/rclcpp.hpp>
#include "rclcpp/time.hpp"
#include <cstdint>
#include <cmath>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

#include "cones_localization/visibility_control.hpp"

#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>

#include "cones_localization/cones_localization.hpp"
#include "cones_interfaces/msg/bounding_box.hpp" 
#include "cones_interfaces/msg/cones.hpp" 


namespace cones_localization
{

class CONES_LOCALIZATION_PUBLIC ConesLocalization
{
public:
  ConesLocalization();

  void lidarProcessing(std::shared_ptr<const sensor_msgs::msg::LaserScan> msg,
                        float fx, float cx,
                        float camera_fov_horizontal,
                        float image_height,
                        float car_yaw_velocity);
  void bboxesProcessing(std::shared_ptr<const cones_interfaces::msg::Cones> msg);
  void imageProcessing(std::shared_ptr<const sensor_msgs::msg::Image> msg);
  nav_msgs::msg::OccupancyGrid::SharedPtr localizationProcessing(geometry_msgs::msg::Pose msg,
                                                                  const nav_msgs::msg::OccupancyGrid::SharedPtr msg_map,
                                                                  double car_yaw);

  std::unique_ptr<cones_interfaces::msg::Cones> cones_ = std::make_unique<cones_interfaces::msg::Cones>();
  std::vector<std::tuple<float, float>> bboxes_points_;

  void setConfig(int conesNumberMap,
                  float conesShiftFactor);

private:
  std::vector<std::tuple<float, float, float, float>> lidar_points_;
  float max_range_;
  std::vector<std::tuple<float, float, char>> cones_distances_;
  int cones_number_map_;
  float cones_shift_factor_;
};

}  // namespace cones_localization

#endif  // CONES_LOCALIZATION__CONES_LOCALIZATION_HPP_
