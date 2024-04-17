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
  int64_t foo(int64_t bar) const;

  void lidarProcessing(std::shared_ptr<const sensor_msgs::msg::LaserScan> msg,
                        float fx, float cx,
                        float camera_fov_horizontal, float image_height);
  void bboxesProcessing(std::shared_ptr<const cones_interfaces::msg::Cones> msg);
  void imageProcessing(std::shared_ptr<const sensor_msgs::msg::Image>  msg);
  std::shared_ptr<nav_msgs::msg::OccupancyGrid> localizationProcessing(const geometry_msgs::msg::PoseStamped::SharedPtr msg,
                                                      const nav_msgs::msg::OccupancyGrid::SharedPtr msg_map);

  std::unique_ptr<cones_interfaces::msg::Cones> cones_ = std::make_unique<cones_interfaces::msg::Cones>();
  std::vector<std::tuple<float, float>> bboxes_points_;

private:
  std::vector<std::tuple<float, float, float, float>> lidar_points_;
  float max_range_;
  std::vector<std::tuple<float, float>> cones_distances_;
};

}  // namespace cones_localization

#endif  // CONES_LOCALIZATION__CONES_LOCALIZATION_HPP_
