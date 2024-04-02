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

#ifndef CONES_LOCALIZATION__CONES_LOCALIZATION_NODE_HPP_
#define CONES_LOCALIZATION__CONES_LOCALIZATION_NODE_HPP_

#include <memory>
#include <rclcpp/rclcpp.hpp>

#include "cones_localization/cones_localization.hpp"

namespace cones_localization
{
using ConesLocalizationPtr = std::unique_ptr<cones_localization::ConesLocalization>;

class CONES_LOCALIZATION_PUBLIC ConesLocalizationNode : public rclcpp::Node
{
public:
  explicit ConesLocalizationNode(const rclcpp::NodeOptions & options);

private:
  ConesLocalizationPtr cones_localization_{nullptr};
  int64_t param_name_{123};
};
}  // namespace cones_localization

#endif  // CONES_LOCALIZATION__CONES_LOCALIZATION_NODE_HPP_
