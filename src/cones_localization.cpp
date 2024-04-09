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

#include "cones_localization/cones_localization.hpp"

#include <iostream>

namespace cones_localization
{

ConesLocalization::ConesLocalization()
{
}

int64_t ConesLocalization::foo(int64_t bar) const
{
  std::cout << "Hello World, " << bar << std::endl;
  return bar;
}

std::vector<std::tuple<float, float, float>> ConesLocalization::lidarProcessing(const sensor_msgs::msg::LaserScan::SharedPtr msg,
                                        float fx_, float cx_, float camera_fov_horizontal_, float image_height_)
  {
    std::cout << "Lidar processing " << std::endl;
    std::cout << msg->header.frame_id << std::endl;

    auto scan_msg = std::make_unique<sensor_msgs::msg::LaserScan>();
    scan_msg->header = msg->header;

    scan_msg->angle_min = msg->angle_min;
    scan_msg->angle_max = msg->angle_max;
    scan_msg->angle_increment = msg->angle_increment;
    scan_msg->time_increment = msg->time_increment;
    scan_msg->scan_time = msg->scan_time;
    scan_msg->range_min = msg->range_min;
    scan_msg->range_max = msg->range_max;

    // determine amount of rays to create
    uint32_t ranges_size = std::ceil(
      (scan_msg->angle_max - scan_msg->angle_min) / scan_msg->angle_increment);

    // std::cout << "ranges_size: " << ranges_size << std::endl;


    lidar_points_.clear();

    float min_angle = msg->angle_min;
    float max_angle = msg->angle_max;
    float angle_increment = msg->angle_increment;

    // float lidar_fov = max_angle - min_angle; 

    std::vector<float> ranges = msg->ranges;

    const float camera_offset = 0.1;
    max_range_ = msg->range_max;
    int y_pixel;
    int x_pixel;
    float x_camera;
    float normalized_angle;
    float alpha;
    float angle;
    float distance;

    for (uint32_t i = 0; i < ranges_size; ++i)
    { 
      angle = max_angle - i * angle_increment;    
      distance = ranges[i];

      // Check if point is within camera FOV 
      if (angle >= min_angle && angle <= max_angle && angle <= camera_fov_horizontal_/2 && angle >= -camera_fov_horizontal_/2)
      {
          // Normalize angle to be between 0 and 1 relative to camera FOV
          normalized_angle = ((angle+(camera_fov_horizontal_/2)) / (camera_fov_horizontal_));

          x_camera = tan(normalized_angle * camera_fov_horizontal_ - camera_fov_horizontal_ / 2);
          x_pixel = static_cast<int>(fx_ * x_camera + cx_);

          if (distance >= max_range_) {
              y_pixel = image_height_ / 2;
              continue;
          }

          alpha = atan(distance / camera_offset);

          y_pixel = static_cast<int>((alpha/1.5708) * image_height_/2);

          lidar_points_.emplace_back(x_pixel, y_pixel, distance);
      }  
    }

    return lidar_points_;
  }

std::unique_ptr<cones_interfaces::msg::Cones> ConesLocalization::bboxesProcessing(const cones_interfaces::msg::Cones::SharedPtr msg)
{
  std::cout << "Bboxes processing" <<std::endl;
  std::cout << msg->header.frame_id << std::endl;
}

void ConesLocalization::imageProcessing(const sensor_msgs::msg::Image::SharedPtr msg,
                                        std::vector<std::tuple<float, float, float>> lidar_points_,
                                        std::unique_ptr<cones_interfaces::msg::Cones> cones_test_)
{
  std::cout << "Image processing" <<std::endl;
  std::cout << msg->header.frame_id << std::endl;
}

void ConesLocalization::localizationProcessing(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
  std::cout << "Localization processing" << std::endl;
  std::cout << msg->header.frame_id << std::endl;
}

}  // namespace cones_localization
