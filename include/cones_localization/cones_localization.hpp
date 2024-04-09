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

#include "cones_localization/visibility_control.hpp"

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

class CONES_LOCALIZATION_PUBLIC ConesLocalization
{
public:
  ConesLocalization();
  int64_t foo(int64_t bar) const;

  std::vector<std::tuple<float, float, float>> lidarProcessing(const sensor_msgs::msg::LaserScan::SharedPtr msg,
                        float fx_, float cx_, float camera_fov_horizontal_, float image_height_);
  std::unique_ptr<cones_interfaces::msg::Cones> bboxesProcessing(const cones_interfaces::msg::Cones::SharedPtr msg);
  void imageProcessing(const sensor_msgs::msg::Image::SharedPtr msg,
                        std::vector<std::tuple<float, float, float>> lidar_points_,
                        std::unique_ptr<cones_interfaces::msg::Cones> cones_test_);
  void localizationProcessing(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);

private:
  std::vector<std::tuple<float, float, float>> lidar_points_;
  float max_range_;
};

}  // namespace cones_localization

#endif  // CONES_LOCALIZATION__CONES_LOCALIZATION_HPP_
