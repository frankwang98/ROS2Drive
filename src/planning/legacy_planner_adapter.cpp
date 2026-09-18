#include "planning/legacy_planner_adapter.hpp"

#include <algorithm>
#include <cmath>

namespace sdc::planning {

LegacyPlannerAdapter::LegacyPlannerAdapter(Type type) : type_(type) {}

PlanningResult LegacyPlannerAdapter::plan(const PlanningInput& input) {
  PlanningResult result;
  if (!input.vehicle.localized) { result.reason = "vehicle_not_localized"; return result; }
  std::vector<sdc::Obstacle> obstacles;
  obstacles.reserve(input.obstacles.size());
  for (const auto& obstacle : input.obstacles)
    obstacles.push_back({{obstacle.pose.x, obstacle.pose.y}, obstacle.radius});

  std::vector<LatticeTrajectory> candidates;
  if (type_ == Type::kLattice) {
    lattice_.plan(input.vehicle.pose.x, input.vehicle.pose.y,
                  input.vehicle.pose.yaw, obstacles, candidates);
  } else {
    em_.plan(input.vehicle.pose.x, input.vehicle.pose.y,
             input.vehicle.pose.yaw, obstacles, candidates);
  }
  const auto selected = std::find_if(candidates.begin(), candidates.end(),
                                     [](const auto& candidate) { return candidate.selected; });
  if (selected == candidates.end() || selected->path.size() < 2) {
    result.reason = "legacy_planner_no_candidate";
    return result;
  }
  result.trajectory.frame_id = "map";
  result.trajectory.stamp_s = input.now_s;
  const double speed = std::min(input.speed_limit,
                                selected->speed > 0.0 ? selected->speed : input.speed_limit);
  for (std::size_t i = 0; i < selected->path.size(); ++i) {
    domain::Waypoint waypoint;
    waypoint.pose.x = selected->path[i].x;
    waypoint.pose.y = selected->path[i].y;
    waypoint.pose.z = 0.0;
    if (i + 1 < selected->path.size())
      waypoint.pose.yaw = std::atan2(selected->path[i + 1].y - selected->path[i].y,
                                     selected->path[i + 1].x - selected->path[i].x);
    else
      waypoint.pose.yaw = i > 0 ? result.trajectory.points.back().pose.yaw
                                : input.vehicle.pose.yaw;
    waypoint.velocity = speed;
    waypoint.relative_time = i * 0.1;
    result.trajectory.points.push_back(waypoint);
  }
  result.trajectory.valid = true;
  result.success = true;
  result.reason = "ok";
  return result;
}

}  // namespace sdc::planning
