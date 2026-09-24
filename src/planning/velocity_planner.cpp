#include "planning/velocity_planner.hpp"

#include <algorithm>
#include <cmath>

namespace sdc::planning {
namespace {
double distance(const domain::Waypoint& a, const domain::Waypoint& b) {
  return std::hypot(a.pose.x - b.pose.x, a.pose.y - b.pose.y);
}
}  // namespace

VelocityPlanner::VelocityPlanner() = default;
VelocityPlanner::VelocityPlanner(Config config) : config_(config) {}

void VelocityPlanner::apply(double speed_limit, domain::Trajectory& trajectory) const {
  auto& points = trajectory.points;
  if (points.empty())
    return;
  const double bounded_speed_limit = std::max(0.0, speed_limit);

  for (auto& point : points) {
    const double curve_limit = point.curvature == 0.0
                                   ? bounded_speed_limit
                                   : std::sqrt(config_.maximum_lateral_acceleration /
                                               std::max(1e-4, std::abs(point.curvature)));
    point.velocity =
        point.stop_required
            ? 0.0
            : std::min(bounded_speed_limit, std::max(config_.minimum_curve_speed, curve_limit));
  }

  for (std::size_t i = 1; i < points.size(); ++i) {
    const double ds = distance(points[i - 1], points[i]);
    const double reachable = std::sqrt(std::max(
        0.0,
        points[i - 1].velocity * points[i - 1].velocity + 2.0 * config_.maximum_acceleration * ds));
    points[i].velocity = std::min(points[i].velocity, reachable);
  }
  for (std::size_t i = points.size() - 1; i > 0; --i) {
    const double ds = distance(points[i - 1], points[i]);
    const double reachable = std::sqrt(std::max(
        0.0, points[i].velocity * points[i].velocity + 2.0 * config_.maximum_deceleration * ds));
    points[i - 1].velocity = std::min(points[i - 1].velocity, reachable);
  }

  double relative_time = 0.0;
  points.front().relative_time = 0.0;
  for (std::size_t i = 1; i < points.size(); ++i) {
    const double average_speed = std::max(0.1, 0.5 * (points[i - 1].velocity + points[i].velocity));
    relative_time += distance(points[i - 1], points[i]) / average_speed;
    points[i].relative_time = relative_time;
    const double dt = std::max(1e-3, points[i].relative_time - points[i - 1].relative_time);
    points[i].acceleration = (points[i].velocity - points[i - 1].velocity) / dt;
  }
}

}  // namespace sdc::planning
