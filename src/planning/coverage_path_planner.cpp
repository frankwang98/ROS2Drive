#include "planning/coverage_path_planner.hpp"

#include <algorithm>
#include <cmath>

namespace sdc::planning {

CoveragePathPlanner::CoveragePathPlanner()
    : CoveragePathPlanner(Config{}) {}

CoveragePathPlanner::CoveragePathPlanner(Config config)
    : config_(config), dense_planner_(ReferencePathPlanner::Config{
          config.spacing, config.horizon, config.obstacle_margin, false}) {}

std::vector<domain::Pose2D> CoveragePathPlanner::build_rows() const {
  std::vector<domain::Pose2D> route;
  const int rows = std::max(1, static_cast<int>(std::floor(
      (config_.max_y - config_.min_y) / config_.row_spacing)) + 1);
  const double radius = std::max(1.0, config_.headland_radius);
  for (int row = 0; row < rows; ++row) {
    const double y = config_.min_y + row * config_.row_spacing;
    const bool left_to_right = (row % 2) == 0;
    if (left_to_right) {
      route.push_back({config_.min_x, y, 0.0, 0.0});
      route.push_back({config_.max_x, y, 0.0, 0.0});
      if (row + 1 < rows) {
        const double next_y = y + config_.row_spacing;
        route.push_back({config_.max_x + radius * 0.75, y + radius * 0.30, 0.4, 0.0});
        route.push_back({config_.max_x + radius, y + radius, 1.57, 0.0});
        route.push_back({config_.max_x + radius * 0.75, next_y - radius * 0.30, 2.7, 0.0});
        route.push_back({config_.max_x, next_y, 3.14159, 0.0});
      }
    } else {
      route.push_back({config_.max_x, y, 3.14159, 0.0});
      route.push_back({config_.min_x, y, 3.14159, 0.0});
      if (row + 1 < rows) {
        const double next_y = y + config_.row_spacing;
        route.push_back({config_.min_x - radius * 0.75, y + radius * 0.30, 2.7, 0.0});
        route.push_back({config_.min_x - radius, y + radius, 1.57, 0.0});
        route.push_back({config_.min_x - radius * 0.75, next_y - radius * 0.30, 0.4, 0.0});
        route.push_back({config_.min_x, next_y, 0.0, 0.0});
      }
    }
  }
  route.push_back({0.0, config_.max_y + radius + 2.0, 0.0, 0.0});
  return route;
}

PlanningResult CoveragePathPlanner::plan(const PlanningInput& input) {
  PlanningInput generated = input;
  generated.reference_path = build_rows();
  return dense_planner_.plan(generated);
}

}  // namespace sdc::planning
