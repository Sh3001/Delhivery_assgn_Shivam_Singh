// Copyright 2026 Delhivery RSE Assignment
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef AMR_NAVIGATION__RAMP_COST_LAYER_HPP_
#define AMR_NAVIGATION__RAMP_COST_LAYER_HPP_

#include <string>
#include <vector>

#include "nav2_costmap_2d/layer.hpp"
#include "rclcpp/rclcpp.hpp"

namespace amr_navigation
{

enum class RegionKind
{
  Traversable,  ///< passable at an elevated cost (a ramp)
  Blocked,      ///< impassable; stamped LETHAL (a deck edge or kerb)
};

struct RampRegion
{
  double x_min;
  double y_min;
  double x_max;
  double y_max;
  unsigned char cost;
  RegionKind kind{RegionKind::Traversable};

  bool contains(double x, double y) const
  {
    return x >= x_min && x <= x_max && y >= y_min && y <= y_max;
  }
};

class RampCostLayer : public nav2_costmap_2d::Layer
{
public:
  RampCostLayer() = default;

  void onInitialize() override;
  void updateBounds(
    double robot_x, double robot_y, double robot_yaw,
    double * min_x, double * min_y, double * max_x, double * max_y) override;
  void updateCosts(
    nav2_costmap_2d::Costmap2D & master_grid,
    int min_i, int min_j, int max_i, int max_j) override;
  void reset() override;
  bool isClearable() override {return false;}

private:
  std::vector<RampRegion> parseRegions(
    const std::vector<double> & flat, RegionKind kind, double default_cost,
    const std::string & param_name) const;

  std::vector<RampRegion> ramp_regions_;
  std::vector<RampRegion> blocked_regions_;
  unsigned char unknown_space_cost_ = 0;
  bool treat_unknown_as_cost_ = false;
};

}  // namespace amr_navigation

#endif  // AMR_NAVIGATION__RAMP_COST_LAYER_HPP_
