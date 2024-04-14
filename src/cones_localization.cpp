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

void ConesLocalization::lidarProcessing(const sensor_msgs::msg::LaserScan::SharedPtr msg,
                                        float fx, float cx,
                                        float camera_fov_horizontal, float image_height)
  {
    lidar_points_.clear();

    float min_angle = msg->angle_min;
    float max_angle = msg->angle_max;
    float angle_increment = msg->angle_increment;

    uint32_t ranges_size = std::ceil(
      (max_angle - min_angle) / angle_increment);

    // float lidar_fov = max_angle - min_angle; 

    std::vector<float> ranges = msg->ranges;

    const float camera_offset = 0.1;
    max_range_ = msg->range_max;
    int y_pixel;
    int x_pixel;
    float x_camera;
    // float normalized_angle;
    float alpha;
    float angle;
    float distance;

    for (uint32_t i = 0; i < ranges_size; ++i)
    { 
      angle = max_angle - i * angle_increment;    
      distance = ranges[i];

      // Check if point is within camera FOV 
      if (angle >= min_angle && angle <= max_angle && angle <= camera_fov_horizontal/2 && angle >= -camera_fov_horizontal/2)
      {
          x_camera = tan(angle);
          x_pixel = static_cast<int>(fx * x_camera + cx);

          if (distance >= max_range_) {
              y_pixel = image_height / 2;
              continue;
          }

          alpha = atan(distance / camera_offset);

          y_pixel = static_cast<int>((alpha/(0.5 * M_PI)) * image_height/2);

          lidar_points_.emplace_back(x_pixel, y_pixel, distance, angle);
      }  
    }
  }

void ConesLocalization::bboxesProcessing(const cones_interfaces::msg::Cones::SharedPtr msg)
{
  cones_->header = msg->header;
  cones_->bboxes = msg->bboxes;
}

void ConesLocalization::imageProcessing(const sensor_msgs::msg::Image::SharedPtr msg)
{
  cv::Mat frame_cv;
  frame_cv = cv_bridge::toCvCopy(msg, "bgr8")->image;

  cones_distances_.clear();
  
  std::sort(cones_->bboxes.begin(), cones_->bboxes.end(), [](const cones_interfaces::msg::BoundingBox& bbox1, const cones_interfaces::msg::BoundingBox& bbox2) {
    return (bbox1.x2 - bbox1.x1) * (bbox1.y2 - bbox1.y1) > (bbox2.x2 - bbox2.x1) * (bbox2.y2 - bbox2.y1);
  });

  int num_bboxes = std::min(static_cast<int>(cones_->bboxes.size()), 3);

  for (int i = 0; i < num_bboxes; ++i)
  {
    const auto& bbox = cones_->bboxes[i];
    bboxes_points_.clear();

    for (const auto& point : lidar_points_)
    {
      if(std::get<0>(point) >= bbox.x1 && std::get<0>(point) <= bbox.x2)
      {
        cv::circle(frame_cv, cv::Point(std::get<0>(point), std::get<1>(point)), 2, cv::Scalar(0, 0, 255), -1);
        bboxes_points_.push_back(std::make_tuple(std::get<2>(point), std::get<3>(point)));
      }
    }
    if (!bboxes_points_.empty())
    {
      std::sort(bboxes_points_.begin(), bboxes_points_.end());

      float median_distance;
      float median_angle;
      if (bboxes_points_.size() % 2 == 0)
      {
        median_distance = (std::get<0>(bboxes_points_[bboxes_points_.size() / 2 - 1]) + std::get<0>(bboxes_points_[bboxes_points_.size() / 2])) / 2.0;
        median_angle = (std::get<1>(bboxes_points_[bboxes_points_.size() / 2 - 1]) + std::get<1>(bboxes_points_[bboxes_points_.size() / 2])) / 2.0;
      }
      else
      {
        median_distance = std::get<0>(bboxes_points_[bboxes_points_.size() / 2]);
        median_angle = std::get<1>(bboxes_points_[bboxes_points_.size() / 2]);
      }

      float rounded_median = std::round(median_distance * 100) / 100;
      std::stringstream ss;
      ss << std::fixed << std::setprecision(2) << rounded_median;
      cv::putText(frame_cv, ss.str(), cv::Point(bbox.x2, bbox.y1), cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0, 0, 255), 2);
      
      cones_distances_.push_back(std::make_tuple(rounded_median, median_angle));
    }
    cv::imshow("Cones from localization", frame_cv);
    cv::waitKey(1);
  }
}

std::shared_ptr<nav_msgs::msg::OccupancyGrid> ConesLocalization::localizationProcessing(const geometry_msgs::msg::PoseStamped::SharedPtr msg,
                                                const nav_msgs::msg::OccupancyGrid::SharedPtr msg_map)
{
  std::cout << "Message data: " << msg->header.frame_id << std::endl;

  if (msg_map){
    std::cout << "Message data: " << msg_map->header.stamp.sec << std::endl;

    nav_msgs::msg::OccupancyGrid::SharedPtr msg_map_copy = std::make_shared<nav_msgs::msg::OccupancyGrid>(*msg_map);

    for (const auto& cone : cones_distances_)
    {
      float distance = std::get<0>(cone);
      float angle = std::get<1>(cone);

      float cone_x = msg->pose.position.x + distance * cos(angle);
      float cone_y = msg->pose.position.y + distance * sin(angle);

      unsigned int cone_grid_x = static_cast<unsigned int>((cone_x - msg_map_copy->info.origin.position.x) / msg_map_copy->info.resolution);
      unsigned int cone_grid_y = static_cast<unsigned int>((cone_y - msg_map_copy->info.origin.position.y) / msg_map_copy->info.resolution);

      std::cout << "Cone grid x: " << cone_grid_x << std::endl;
      std::cout << "Cone grid y: " << cone_grid_y << std::endl;

      int radius = 1;
      for (int i = -radius; i <= radius; ++i) {
        for (int j = -radius; j <= radius; ++j) {
          unsigned int grid_x = cone_grid_x + i;
          unsigned int grid_y = cone_grid_y + j;

          if (grid_x < msg_map_copy->info.width && grid_y < msg_map_copy->info.height) {
              int index = grid_y * msg_map_copy->info.width + grid_x;
              msg_map_copy->data[index] = 100;
          }
        }
      }
    }
    return msg_map_copy;
  }
  return msg_map;
}

}  // namespace cones_localization
