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

#include "amr_navigation/ramp_cost_layer.hpp"

#include <algorithm>
#include <cmath>

#include "nav2_costmap_2d/costmap_math.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace amr_navigation
{

namespace
{
constexpr size_t kTraversableStride = 5;  // x_min, y_min, x_max, y_max, cost
constexpr size_t kBlockedStride = 4;      // x_min, y_min, x_max, y_max

size_t strideFor(RegionKind kind)
{
  return kind == RegionKind::Traversable ? kTraversableStride : kBlockedStride;
}
}  // namespace

std::vector<RampRegion> RampCostLayer::parseRegions(
  const std::vector<double> & flat, RegionKind kind, double default_cost,
  const std::string & param_name) const
{
  const size_t stride = strideFor(kind);
  std::vector<RampRegion> regions;

  if (stride != 0 && flat.size() % stride != 0) {
    RCLCPP_ERROR(
      logger_,
      "RampCostLayer '%s': %s has %zu values, which is not a multiple of %zu. "
      "The trailing %zu value(s) describe an incomplete region and are ignored; "
      "fix the configuration.",
      name_.c_str(), param_name.c_str(), flat.size(), stride, flat.size() % stride);
  }

  for (size_t i = 0; i + stride <= flat.size(); i += stride) {
    RampRegion region;
    region.x_min = std::min(flat[i], flat[i + 2]);
    region.x_max = std::max(flat[i], flat[i + 2]);
    region.y_min = std::min(flat[i + 1], flat[i + 3]);
    region.y_max = std::max(flat[i + 1], flat[i + 3]);
    region.kind = kind;

    if (kind == RegionKind::Blocked) {
      region.cost = nav2_costmap_2d::LETHAL_OBSTACLE;
    } else {
      double cost = flat[i + 4];
      if (cost <= 0.0) {
        cost = default_cost;
      }
      region.cost = static_cast<unsigned char>(
        std::clamp(
          cost, 1.0, static_cast<double>(nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE - 1)));
    }
    regions.push_back(region);
  }
  return regions;
}

void RampCostLayer::onInitialize()
{
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error("RampCostLayer: unable to lock node");
  }

  declareParameter("enabled", rclcpp::ParameterValue(true));
  declareParameter("ramp_regions", rclcpp::ParameterValue(std::vector<double>{}));
  declareParameter("default_cost", rclcpp::ParameterValue(200.0));
  declareParameter("blocked_regions", rclcpp::ParameterValue(std::vector<double>{}));
  declareParameter("unknown_space_cost", rclcpp::ParameterValue(0.0));

  node->get_parameter(name_ + ".enabled", enabled_);

  std::vector<double> flat_regions;
  node->get_parameter(name_ + ".ramp_regions", flat_regions);
  double default_cost = 200.0;
  node->get_parameter(name_ + ".default_cost", default_cost);

  double unknown_space_cost = 0.0;
  node->get_parameter(name_ + ".unknown_space_cost", unknown_space_cost);
  treat_unknown_as_cost_ = unknown_space_cost > 0.0;
  unknown_space_cost_ = static_cast<unsigned char>(
    std::clamp(
      unknown_space_cost, 1.0,
      static_cast<double>(nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE - 1)));

  ramp_regions_ = parseRegions(
    flat_regions, RegionKind::Traversable, default_cost, "ramp_regions");
  RCLCPP_INFO(
    logger_, "RampCostLayer '%s': loaded %zu ramp region(s)", name_.c_str(), ramp_regions_.size());

  std::vector<double> flat_blocked;
  node->get_parameter(name_ + ".blocked_regions", flat_blocked);
  blocked_regions_ = parseRegions(
    flat_blocked, RegionKind::Blocked, default_cost, "blocked_regions");
  RCLCPP_INFO(
    logger_, "RampCostLayer '%s': loaded %zu blocked (elevated) region(s)",
    name_.c_str(), blocked_regions_.size());

  current_ = true;
}

void RampCostLayer::updateBounds(
  double /*robot_x*/, double /*robot_y*/, double /*robot_yaw*/,
  double * min_x, double * min_y, double * max_x, double * max_y)
{
  if (!enabled_) {
    return;
  }
  for (const auto & region : ramp_regions_) {
    *min_x = std::min(*min_x, region.x_min);
    *min_y = std::min(*min_y, region.y_min);
    *max_x = std::max(*max_x, region.x_max);
    *max_y = std::max(*max_y, region.y_max);
  }
  for (const auto & region : blocked_regions_) {
    *min_x = std::min(*min_x, region.x_min);
    *min_y = std::min(*min_y, region.y_min);
    *max_x = std::max(*max_x, region.x_max);
    *max_y = std::max(*max_y, region.y_max);
  }

  if (treat_unknown_as_cost_) {
    nav2_costmap_2d::Costmap2D * costmap = layered_costmap_->getCostmap();
    double origin_x = costmap->getOriginX();
    double origin_y = costmap->getOriginY();
    double size_x = costmap->getSizeInCellsX() * costmap->getResolution();
    double size_y = costmap->getSizeInCellsY() * costmap->getResolution();
    *min_x = std::min(*min_x, origin_x);
    *min_y = std::min(*min_y, origin_y);
    *max_x = std::max(*max_x, origin_x + size_x);
    *max_y = std::max(*max_y, origin_y + size_y);
  }
}

void RampCostLayer::updateCosts(
  nav2_costmap_2d::Costmap2D & master_grid,
  int min_i, int min_j, int max_i, int max_j)
{
  if (!enabled_) {
    return;
  }

  for (const auto & region : blocked_regions_) {
    unsigned int bsx, bsy, bex, bey;
    if (!master_grid.worldToMap(region.x_min, region.y_min, bsx, bsy)) {continue;}
    if (!master_grid.worldToMap(region.x_max, region.y_max, bex, bey)) {continue;}
    for (int iy = std::max(static_cast<int>(bsy), min_j);
      iy < std::min(static_cast<int>(bey), max_j); ++iy)
    {
      for (int ix = std::max(static_cast<int>(bsx), min_i);
        ix < std::min(static_cast<int>(bex), max_i); ++ix)
      {
        master_grid.setCost(ix, iy, nav2_costmap_2d::LETHAL_OBSTACLE);
      }
    }
  }

  for (const auto & region : ramp_regions_) {
    unsigned int start_x, start_y, end_x, end_y;
    if (!master_grid.worldToMap(region.x_min, region.y_min, start_x, start_y)) {
      continue;
    }
    if (!master_grid.worldToMap(region.x_max, region.y_max, end_x, end_y)) {
      continue;
    }

    int ix_min = std::max(static_cast<int>(start_x), min_i);
    int iy_min = std::max(static_cast<int>(start_y), min_j);
    int ix_max = std::min(static_cast<int>(end_x), max_i);
    int iy_max = std::min(static_cast<int>(end_y), max_j);

    for (int iy = iy_min; iy < iy_max; ++iy) {
      for (int ix = ix_min; ix < ix_max; ++ix) {
        master_grid.setCost(ix, iy, region.cost);
      }
    }
  }

  if (treat_unknown_as_cost_) {
    for (int iy = min_j; iy < max_j; ++iy) {
      for (int ix = min_i; ix < max_i; ++ix) {
        if (master_grid.getCost(ix, iy) == nav2_costmap_2d::NO_INFORMATION) {
          master_grid.setCost(ix, iy, unknown_space_cost_);
        }
      }
    }
  }
}

void RampCostLayer::reset()
{
  current_ = true;
}

}  // namespace amr_navigation

PLUGINLIB_EXPORT_CLASS(amr_navigation::RampCostLayer, nav2_costmap_2d::Layer)
