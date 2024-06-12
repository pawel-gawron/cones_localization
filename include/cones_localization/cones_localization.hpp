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

#include "kalman_filter.hpp"

#include <kalman_filter/kalman_filter.hpp>


namespace cones_localization
{

typedef struct {
  float x;
  float y;
  double angle;
  char label;
  float idx;
} PointXYI;

void cones_draw(double x_factor,
                double y_factor,
                std::shared_ptr<nav_msgs::msg::OccupancyGrid> msg_map_copy,
                std::vector<PointXYI> cones_posisiton_,
                int cones_diameter_on_map);

void cones_buffor(std::vector<std::tuple<float, float, char>> cones_distances_,
                  geometry_msgs::msg::Pose msg_loc,
                  double car_yaw,
                  std::vector<PointXYI>& cones_posisiton_);

class CONES_LOCALIZATION_PUBLIC ConesLocalization
{
  public:
    ConesLocalization();

    void lidarProcessing(std::shared_ptr<const sensor_msgs::msg::LaserScan> msg_lidar,
                          float fx, float cx,
                          float camera_fov_horizontal,
                          float image_height,
                          float car_yaw_velocity);
    void bboxesProcessing(std::shared_ptr<const cones_interfaces::msg::Cones> msg_bboxes);
    void imageProcessing(std::shared_ptr<const sensor_msgs::msg::Image> msg_image, float dt);
    nav_msgs::msg::OccupancyGrid::SharedPtr localizationProcessing(geometry_msgs::msg::Pose msg_loc,
                                                                    const nav_msgs::msg::OccupancyGrid::SharedPtr msg_map,
                                                                    double car_yaw);

    std::unique_ptr<cones_interfaces::msg::Cones> cones_ = std::make_unique<cones_interfaces::msg::Cones>();
    std::vector<std::tuple<float, float>> bboxes_points_;

    void setConfig(int conesNumberMap,
                  float conesShiftFactor,
                  float conesDistanceMeasurement,
                  bool kalmanOn,
                  float kalmanMeasVariance,
                  int cones_diameter_on_map);

    std::unique_ptr<KalmanFilter> kf_distance;
    std::unique_ptr<KalmanFilter> kf_angle;

  private:
    std::vector<std::tuple<float, float, float, float>> lidar_points_;
    float max_range_;
    std::vector<std::tuple<float, float, char>> cones_distances_;

    int cones_number_map_;
    float cones_shift_factor_;
    float cones_distance_measurement_;
    bool kalman_on_;
    float kalman_meas_variance_;
    int cones_diameter_on_map_;
    bool save_obstacle = false;

    char previous_cone_label = '\0';
    char previous_obstacle_cone_label = '\0';

    std::vector<PointXYI> cones_posisiton_;
};

}  // namespace cones_localization

#endif  // CONES_LOCALIZATION__CONES_LOCALIZATION_HPP_
