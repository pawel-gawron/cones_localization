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

void ConesLocalization::lidarProcessing(std::shared_ptr<const sensor_msgs::msg::LaserScan> msg,
                                        float fx, float cx,
                                        float camera_fov_horizontal, float image_height,
                                        float car_yaw_velocity)
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
        float angular_displacement = sin(car_yaw_velocity) * cones_shift_factor_;
        angle = angle - angular_displacement;

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

void ConesLocalization::bboxesProcessing(std::shared_ptr<const cones_interfaces::msg::Cones> msg)
{
  cones_->header = msg->header;
  cones_->bboxes = msg->bboxes;
}

void ConesLocalization::imageProcessing(std::shared_ptr<const sensor_msgs::msg::Image> msg, float dt)
{
  save_obstacle = false;
  cv::Mat frame_cv;
  frame_cv = cv_bridge::toCvCopy(msg, "bgr8")->image;

  cones_distances_.clear();
  
  std::sort(cones_->bboxes.begin(), cones_->bboxes.end(), [](const cones_interfaces::msg::BoundingBox& bbox1, const cones_interfaces::msg::BoundingBox& bbox2) {
    return (bbox1.x2 - bbox1.x1) * (bbox1.y2 - bbox1.y1) > (bbox2.x2 - bbox2.x1) * (bbox2.y2 - bbox2.y1);
  });

  int num_bboxes = std::min(static_cast<int>(cones_->bboxes.size()), cones_number_map_);

  for (int i = 0; i < num_bboxes; ++i)
  {
    const auto& bbox = cones_->bboxes[i];
    bboxes_points_.clear();

    for (const auto& point : lidar_points_)
    {
      int x = std::get<0>(point);
      int y = std::get<1>(point);
      int bbox_width = abs(bbox.x2 - bbox.x1);
      
      if(x >= bbox.x1 - bbox_width / 2 && x <= bbox.x2 + bbox_width / 2)
      {
        cv::circle(frame_cv, cv::Point(x, y), 2, cv::Scalar(0, 0, 255), -1);
        bboxes_points_.push_back(std::make_tuple(std::get<2>(point), std::get<3>(point)));
      }
    }
    if (!bboxes_points_.empty())
    {
      std::sort(bboxes_points_.begin(), bboxes_points_.end());
      
      float median_angle, median_angle_kalman_filter;
      float min_distance, min_distance_kalman_filter;
      char cone_label = bbox.label;

      auto min_distance_it = std::min_element(bboxes_points_.begin(), bboxes_points_.end(),
      [](const auto& a, const auto& b) { return std::get<0>(a) < std::get<0>(b); });
      min_distance = std::get<0>(*min_distance_it);

      if (std::isnan(min_distance)) {
        continue;
      }

      if (bboxes_points_.size() % 2 == 0)
      {
        median_angle = (std::get<1>(bboxes_points_[bboxes_points_.size() / 2 - 1]) + std::get<1>(bboxes_points_[bboxes_points_.size() / 2])) / 2.0;
      }
      else
      {
        median_angle = std::get<1>(bboxes_points_[bboxes_points_.size() / 2]);
      }

      if (kalman_on_){
        if (cone_label != previous_cone_label) {
          kf_distance->reinitial(min_distance, kf_distance->get_mean()[1]);
          kf_angle->reinitial(median_angle, kf_angle->get_mean()[1]);
        }
        else {
          kf_distance->predict(dt);
          kf_angle->predict(dt);
          kf_distance->update(min_distance, kalman_meas_variance_);
          kf_angle->update(median_angle, kalman_meas_variance_);
        }

        median_angle_kalman_filter = kf_angle->get_mean()[0];
        min_distance_kalman_filter = kf_distance->get_mean()[0];

        min_distance = min_distance_kalman_filter;
        median_angle = median_angle_kalman_filter;
      }

      if (min_distance > cones_distance_measurement_ || previous_cone_label == cone_label) {
        continue;
      }
      
      float rounded_distance = std::round(min_distance * 100) / 100;
      float rounded_angle = std::round(median_angle * 100) / 100;

      std::stringstream ss_distance;
      std::stringstream ss_angle;
      ss_distance << std::fixed << std::setprecision(2) << rounded_distance;
      ss_angle << std::fixed << std::setprecision(2) << rounded_angle;

      cv::putText(frame_cv, ss_distance.str(), cv::Point(bbox.x2, bbox.y1), cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0, 0, 255), 2);
      cv::putText(frame_cv, ss_angle.str(), cv::Point(bbox.x2, bbox.y2), cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0, 0, 255), 2);
      
      cones_distances_.push_back(std::make_tuple(min_distance, median_angle, cone_label));

      if (previous_obstacle_cone_label != cone_label)
      {
          save_obstacle = true;
      }
      previous_obstacle_cone_label = cone_label;
      previous_cone_label = cone_label;
    }
  }
  cv::imshow("Cones from localization", frame_cv);
  cv::waitKey(1);
}

std::shared_ptr<nav_msgs::msg::OccupancyGrid> ConesLocalization::localizationProcessing(geometry_msgs::msg::Pose msg,
                                                                                        const nav_msgs::msg::OccupancyGrid::SharedPtr msg_map,
                                                                                        double car_yaw)
{
  if (!msg_map) { return msg_map; } 

  std_msgs::msg::Header header;
  header.stamp = rclcpp::Clock(RCL_ROS_TIME).now();
  header.frame_id = msg_map->header.frame_id;

  nav_msgs::msg::OccupancyGrid::SharedPtr msg_map_copy = std::make_shared<nav_msgs::msg::OccupancyGrid>(*msg_map);

  msg_map_copy->header = header;

  for (const auto& cone : cones_distances_)
  {
    float distance = std::get<0>(cone);
    float angle = std::get<1>(cone);
    char cone_label = std::get<2>(cone);

    if (std::isnan(distance) || std::isnan(angle)) {
      break;
    }

    // std::cout << "Distance: " << distance << " Angle: " << angle << std::endl;
    // std::cout << "Cone label: " << std::get<2>(cone) << std::endl;

    float cone_x = msg.position.x + (distance * cos(car_yaw - angle));
    float cone_y = msg.position.y + (distance * sin(car_yaw - angle));

    float cone_x_occupied = cone_x;
    float cone_y_occupied = cone_y;

    // std::cout << "msg.position.x: " << msg.position.x << " msg.position.y: " << msg.position.y << std::endl;
    // std::cout << "cone_x: " << cone_x << " cone_y: " << cone_y << std::endl;

    unsigned int grid_x, grid_y;
    float direction;

    direction = cone_label == 'Y' ? M_PI_2 : cone_label == 'B' ? -M_PI_2 : 0;

    float dx = cos(car_yaw + direction);
    float dy = sin(car_yaw + direction);

    double x_factor = 1.0 / msg_map_copy->info.resolution;
    double y_factor = 1.0 / msg_map_copy->info.resolution;
    int last_index = 0;

    while (true) {
        grid_x = (cone_x - msg_map_copy->info.origin.position.x) * x_factor;
        grid_y = (cone_y - msg_map_copy->info.origin.position.y) * y_factor;

        if (grid_x >= msg_map_copy->info.width || 
            grid_y >= msg_map_copy->info.height) {
          break;
        }

        int index = (grid_y * msg_map_copy->info.width) + grid_x;
        if (index == last_index) {
          cone_x += dx * msg_map_copy->info.resolution;
          cone_y += dy * msg_map_copy->info.resolution;
          last_index = index;
          continue;
        }
        if (static_cast<int>(msg_map_copy->data[index]) >= 40) {
          break;
        }

        msg_map_copy->data[index] = 100;

        cone_x += dx * msg_map_copy->info.resolution;
        cone_y += dy * msg_map_copy->info.resolution;

        last_index = index;
    }

    int radius = 3;
    for (int i = -radius; i <= radius; ++i) {
      for (int j = -radius; j <= radius; ++j) {
        grid_x = (cone_x_occupied - msg_map_copy->info.origin.position.x) / msg_map_copy->info.resolution;
        grid_y = (cone_y_occupied - msg_map_copy->info.origin.position.y) / msg_map_copy->info.resolution;

        if (grid_x + j < msg_map_copy->info.width &&
            grid_y + i < msg_map_copy->info.height){
          int index = (grid_y + i) * msg_map_copy->info.width + (grid_x + j);
          msg_map_copy->data[index] = 100;
        }
      }
    }
  }

  if (save_obstacle) {
    *msg_map = *msg_map_copy;
  }

  return msg_map_copy;
}

void ConesLocalization::setConfig(int conesNumberMap,
                                  float conesShiftFactor,
                                  float conesDistanceMeasurement,
                                  bool kalmanOn,
                                  float kalmanMeasVariance)
{
  cones_number_map_ = conesNumberMap;
  cones_shift_factor_ = conesShiftFactor;
  cones_distance_measurement_ = conesDistanceMeasurement;
  kalman_on_ = kalmanOn;
  kalman_meas_variance_ = kalmanMeasVariance;
}

}  // namespace cones_localization
