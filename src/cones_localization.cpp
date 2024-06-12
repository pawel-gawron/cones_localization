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
  : kf_distance(std::make_unique<KalmanFilter>()), kf_angle(std::make_unique<KalmanFilter>())
{
    Eigen::MatrixXd A(2, 2);
    Eigen::MatrixXd B(2, 1);
    Eigen::MatrixXd C(1, 2);
    Eigen::MatrixXd Q(2, 2);
    Eigen::MatrixXd R(1, 1);
    Eigen::MatrixXd P_initial(2, 2);
    Eigen::MatrixXd x_initial(2, 1);

    A << 1, 0,
         0, 1;
    B << 0.5, 1;
    C << 1, 0;
    Q << 0.01, 0,
         0, 0.01;
    R << kalman_meas_variance_;
    P_initial.setIdentity();
    x_initial << 0.0, 0.0;

    kf_distance->init(x_initial, A, B, C, Q, R, P_initial);
    kf_angle->init(x_initial, A, B, C, Q, R, P_initial);
}

void ConesLocalization::lidarProcessing(std::shared_ptr<const sensor_msgs::msg::LaserScan> msg_lidar,
                                        float fx, float cx,
                                        float camera_fov_horizontal, float image_height,
                                        float car_yaw_velocity)
{
  lidar_points_.clear();

  float min_angle = msg_lidar->angle_min;
  float max_angle = msg_lidar->angle_max;
  float angle_increment = msg_lidar->angle_increment;

  uint32_t ranges_size = std::ceil(
    (max_angle - min_angle) / angle_increment);

  std::vector<float> ranges = msg_lidar->ranges;

  const float camera_offset = 0.1;
  max_range_ = msg_lidar->range_max;
  int y_pixel;
  int x_pixel;
  float x_camera;
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

void ConesLocalization::bboxesProcessing(std::shared_ptr<const cones_interfaces::msg::Cones> msg_bboxes)
{
  cones_->header = msg_bboxes->header;
  cones_->bboxes = msg_bboxes->bboxes;
}

void ConesLocalization::imageProcessing(std::shared_ptr<const sensor_msgs::msg::Image> msg_image, float dt)
{
  save_obstacle = false;
  cv::Mat frame_cv = cv_bridge::toCvCopy(msg_image, "bgr8")->image;

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
      int bbox_height = abs(bbox.y2 - bbox.y1);
      
      if(x >= bbox.x1 - bbox_width / 2 &&
          x <= bbox.x2 + bbox_width / 2  &&
          y >= bbox.y1 && y <= bbox.y2 &&
          bbox_width > 0.01 * frame_cv.cols &&
          bbox_height > 0.01 * frame_cv.rows)
      {
        cv::circle(frame_cv, cv::Point(x, y), 2, cv::Scalar(0, 0, 255), -1);
        bboxes_points_.push_back(std::make_tuple(std::get<2>(point), std::get<3>(point)));
      }
    }
    if (!bboxes_points_.empty())
    {
      std::sort(bboxes_points_.begin(), bboxes_points_.end());
      
      char cone_label = bbox.label;

      auto min_distance_it = std::min_element(bboxes_points_.begin(), bboxes_points_.end(),
      [](const auto& a, const auto& b) { return std::get<0>(a) < std::get<0>(b); });
      float min_distance = std::get<0>(*min_distance_it);

      if (std::isnan(min_distance)) {
        continue;
      }

      float median_angle = bboxes_points_.size() % 2 == 0
          ? (std::get<1>(bboxes_points_[bboxes_points_.size() / 2 - 1]) + std::get<1>(bboxes_points_[bboxes_points_.size() / 2])) / 2.0
          : std::get<1>(bboxes_points_[bboxes_points_.size() / 2]);

      if (kalman_on_){
        Eigen::MatrixXd x_initial(2, 1);
        Eigen::MatrixXd P(2, 2);

        if (cone_label != previous_cone_label) {
          kf_distance->getP(P);
          x_initial << min_distance, kf_distance->getXelement(1);
          kf_distance->init(x_initial, P);

          kf_angle->getP(P);
          x_initial << median_angle, kf_angle->getXelement(1);
          kf_angle->init(x_initial, P);
        }
        else {
          Eigen::MatrixXd u(1, 1);
          u << dt;

          kf_distance->predict(u);
          kf_angle->predict(u);

          Eigen::MatrixXd y_distance(1, 1);
          y_distance << min_distance;
          kf_distance->update(y_distance);

          Eigen::MatrixXd y_angle(1, 1);
          y_angle << median_angle;
          kf_angle->update(y_angle);
        }

        Eigen::MatrixXd x_distance;
        kf_distance->getX(x_distance);
        min_distance = x_distance(0, 0);

        Eigen::MatrixXd x_angle;
        kf_angle->getX(x_angle);
        median_angle = x_angle(0, 0);
      }

      if (min_distance > cones_distance_measurement_ || previous_cone_label == cone_label) {
        continue;
      }

      auto to_string_with_precision = [](float value, int precision) {
      std::ostringstream out;
      out << std::fixed << std::setprecision(precision) << value;
      return out.str();
      };

      auto draw_text = [&](const std::string& text, int x, int y) {
          cv::putText(frame_cv, text, cv::Point(x, y), cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0, 0, 255), 2);
      };

      draw_text(to_string_with_precision(min_distance, 2), bbox.x2, bbox.y1);
      draw_text(to_string_with_precision(median_angle, 2), bbox.x2, bbox.y2);
      
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

std::shared_ptr<nav_msgs::msg::OccupancyGrid> ConesLocalization::localizationProcessing(
    geometry_msgs::msg::Pose msg_loc,
    const nav_msgs::msg::OccupancyGrid::SharedPtr msg_map,
    double car_yaw) 
{
    if (!msg_map) {
        return msg_map;
    }

    std_msgs::msg::Header header;
    header.stamp = rclcpp::Clock(RCL_ROS_TIME).now();
    header.frame_id = msg_map->header.frame_id;

    auto msg_map_copy = std::make_shared<nav_msgs::msg::OccupancyGrid>(*msg_map);
    msg_map_copy->header = header;

    cones_buffor(cones_distances_, msg_loc, car_yaw, cones_posisiton_);

    double x_factor = 1.0 / msg_map_copy->info.resolution;
    double y_factor = 1.0 / msg_map_copy->info.resolution;

    cones_draw(x_factor, y_factor, msg_map_copy, cones_posisiton_, cones_diameter_on_map_);

    return msg_map_copy;
}

void ConesLocalization::setConfig(int conesNumberMap,
                                  float conesShiftFactor,
                                  float conesDistanceMeasurement,
                                  bool kalmanOn,
                                  float kalmanMeasVariance,
                                  int cones_diameter_on_map)
{
  cones_number_map_ = conesNumberMap;
  cones_shift_factor_ = conesShiftFactor;
  cones_distance_measurement_ = conesDistanceMeasurement;
  kalman_on_ = kalmanOn;
  kalman_meas_variance_ = kalmanMeasVariance;
  cones_diameter_on_map_ = cones_diameter_on_map;
}

void cones_buffor(std::vector<std::tuple<float, float, char>> cones_distances_,
                  geometry_msgs::msg::Pose msg_loc,
                  double car_yaw,
                  std::vector<PointXYI>& cones_posisiton_)
{
    for (const auto& cone : cones_distances_) {
        float distance = std::get<0>(cone);
        float angle = std::get<1>(cone);
        char cone_label = std::get<2>(cone);

        if (std::isnan(distance) || std::isnan(angle)) {
            break;
        }

        float cone_x = msg_loc.position.x + (distance * cos(car_yaw - angle));
        float cone_y = msg_loc.position.y + (distance * sin(car_yaw - angle));

        auto cone_position = std::find_if(cones_posisiton_.begin(), cones_posisiton_.end(), 
            [&](const PointXYI& cp) {
                return std::sqrt(std::pow(cone_x - cp.x, 2) + std::pow(cone_y - cp.y, 2)) < 0.4 && cone_label == cp.label;
            });

        if (cone_position != cones_posisiton_.end()) {
            cone_position->x = (cone_x / cone_position->idx) + ((cone_position->idx - 1) / cone_position->idx * cone_position->x);
            cone_position->y = (cone_y / cone_position->idx) + ((cone_position->idx - 1) / cone_position->idx * cone_position->y);
            cone_position->angle = (car_yaw / cone_position->idx) + ((cone_position->idx - 1) / cone_position->idx * cone_position->angle);
            cone_position->label = cone_label;
            cone_position->idx += 1;
        } else {
            cones_posisiton_.push_back({cone_x, cone_y, car_yaw, cone_label, 1});
        }
    }
}


void cones_draw(double x_factor,
                double y_factor,
                std::shared_ptr<nav_msgs::msg::OccupancyGrid> msg_map_copy,
                std::vector<PointXYI> cones_posisiton_,
                int cones_diameter_on_map)
{
  for (const auto& cone_position : cones_posisiton_) {
    float cone_x = cone_position.x;
    float cone_y = cone_position.y;
    float direction = cone_position.label == 'Y' ? M_PI_2 : cone_position.label == 'B' ? -M_PI_2 : 0;

    float dx = cos(cone_position.angle + direction);
    float dy = sin(cone_position.angle + direction);
    unsigned int grid_x, grid_y;
    int last_index = -1;

    while (true) {
      grid_x = (cone_x - msg_map_copy->info.origin.position.x) * x_factor;
      grid_y = (cone_y - msg_map_copy->info.origin.position.y) * y_factor;

      if (grid_x >= msg_map_copy->info.width || grid_y >= msg_map_copy->info.height) {
        break;
      }

      int index = (grid_y * msg_map_copy->info.width) + grid_x;
      if (index == last_index) {
        cone_x += dx * msg_map_copy->info.resolution;
        cone_y += dy * msg_map_copy->info.resolution;
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

      grid_x = (cone_position.x - msg_map_copy->info.origin.position.x) * x_factor;
      grid_y = (cone_position.y - msg_map_copy->info.origin.position.y) * y_factor;

      for (int i = -cones_diameter_on_map; i <= cones_diameter_on_map; ++i) {
        for (int j = -cones_diameter_on_map; j <= cones_diameter_on_map; ++j) {
          if (grid_x + j < msg_map_copy->info.width && grid_y + i < msg_map_copy->info.height) {
            int index = (grid_y + i) * msg_map_copy->info.width + (grid_x + j);
            msg_map_copy->data[index] = 100;
          }
        }
      }
  }
}

}  // namespace cones_localization
