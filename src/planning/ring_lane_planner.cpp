#include "planning/ring_lane_planner.hpp"

#include <algorithm>
#include <cmath>
#include <sstream>

namespace sdc::planning {
namespace {
constexpr double kPi = 3.14159265358979323846;

double wrap_positive(double angle) {
  while (angle < 0.0)
    angle += 2.0 * kPi;
  while (angle >= 2.0 * kPi)
    angle -= 2.0 * kPi;
  return angle;
}

double smoothstep(double t) {
  t = std::clamp(t, 0.0, 1.0);
  return t * t * (3.0 - 2.0 * t);
}
}  // namespace

RingLanePlanner::RingLanePlanner() = default;
RingLanePlanner::RingLanePlanner(Config config) : config_(config) {}

PlanningResult RingLanePlanner::plan(const PlanningInput& input) {
  PlanningResult result;
  if (!input.vehicle.localized || input.reference_path.size() < 3) {
    result.reason = "ring_lane_input_invalid";
    return result;
  }

  // The mission route is the left lane centre.  Its mean radial distance
  // makes this independent of the hard-coded 26 m teaching-road radius.
  double left_radius = 0.0;
  for (const auto& pose : input.reference_path)
    left_radius += std::hypot(pose.x, pose.y);
  left_radius /= static_cast<double>(input.reference_path.size());
  const double vehicle_radius = std::hypot(input.vehicle.pose.x, input.vehicle.pose.y);
  const double vehicle_angle = std::atan2(input.vehicle.pose.y, input.vehicle.pose.x);

  bool left_blocked_ahead = false;
  bool left_obstacle_cleared_behind = false;
  bool right_blocked_ahead = false;
  for (const auto& obstacle : input.obstacles) {
    const double obstacle_radius = std::hypot(obstacle.pose.x, obstacle.pose.y);
    const double lateral = obstacle_radius - left_radius;
    const double angular_delta =
        std::atan2(std::sin(std::atan2(obstacle.pose.y, obstacle.pose.x) - vehicle_angle),
                   std::cos(std::atan2(obstacle.pose.y, obstacle.pose.x) - vehicle_angle));
    const double forward_s = wrap_positive(angular_delta) * left_radius;
    const double signed_s = angular_delta * left_radius;
    const bool ahead = forward_s > 0.5 && forward_s < config_.horizon;
    const bool same_left_lane = std::abs(lateral) < config_.lane_width * 0.45;
    const bool same_right_lane = std::abs(lateral - config_.lane_width) < config_.lane_width * 0.45;
    if (same_left_lane && ahead)
      left_blocked_ahead = true;
    if (same_left_lane && signed_s < -6.0)
      left_obstacle_cleared_behind = true;
    if (same_right_lane && ahead)
      right_blocked_ahead = true;
  }

  if (maneuver_ == Maneuver::kKeepLeft && left_blocked_ahead && !right_blocked_ahead)
    maneuver_ = Maneuver::kChangeRight;
  if (maneuver_ == Maneuver::kChangeRight &&
      vehicle_radius >= left_radius + config_.lane_width * 0.85) {
    maneuver_ = Maneuver::kKeepRight;
    passed_left_obstacle_ = false;
  }
  if (maneuver_ == Maneuver::kKeepRight && left_obstacle_cleared_behind)
    passed_left_obstacle_ = true;
  if (maneuver_ == Maneuver::kKeepRight && passed_left_obstacle_ && !left_blocked_ahead)
    maneuver_ = Maneuver::kChangeLeft;
  if (maneuver_ == Maneuver::kChangeLeft &&
      vehicle_radius <= left_radius + config_.lane_width * 0.15)
    maneuver_ = Maneuver::kKeepLeft;

  const double current_lateral = vehicle_radius - left_radius;
  double target_lateral = 0.0;
  if (maneuver_ == Maneuver::kChangeRight || maneuver_ == Maneuver::kKeepRight)
    target_lateral = config_.lane_width;

  const int samples = std::max(2, static_cast<int>(std::ceil(config_.horizon / config_.spacing)));
  result.trajectory.frame_id = "map";
  result.trajectory.stamp_s = input.now_s;
  for (int i = 0; i <= samples; ++i) {
    const double s = config_.horizon * static_cast<double>(i) / samples;
    const double transition =
        (maneuver_ == Maneuver::kChangeRight || maneuver_ == Maneuver::kChangeLeft)
            ? smoothstep(s / config_.change_length)
            : 1.0;
    const double lateral = current_lateral + (target_lateral - current_lateral) * transition;
    const double radius = left_radius + lateral;
    const double angle = vehicle_angle + s / std::max(1.0, radius);
    domain::Waypoint point;
    point.pose.x = radius * std::cos(angle);
    point.pose.y = radius * std::sin(angle);
    point.pose.yaw = angle + kPi / 2.0;
    point.velocity = input.speed_limit;
    point.relative_time = s / std::max(0.1, input.speed_limit);
    result.trajectory.points.push_back(point);
  }

  // A trajectory that intersects an inflated obstacle is never executable.
  // Do not "best effort" through it: VehicleRuntime converts this planning
  // failure into a SafetyManager stop command.
  for (const auto& obstacle : input.obstacles) {
    const double obstacle_angle = std::atan2(obstacle.pose.y, obstacle.pose.x);
    const double forward_s = wrap_positive(obstacle_angle - vehicle_angle) * left_radius;
    // Obstacles behind the rear axle do not collide with a forward-only local
    // trajectory.  Without this filter, an already-passed left-lane blocker
    // falsely vetoes the right-to-left return manoeuvre.
    if (forward_s > config_.horizon + config_.spacing)
      continue;
    const double obstacle_lateral = std::hypot(obstacle.pose.x, obstacle.pose.y) - left_radius;
    // Once there is enough distance to complete the manoeuvre, a blocker in
    // the left lane must not veto a trajectory whose committed target is the
    // right lane.  The old point-wise Euclidean check treated that adjacent
    // lane blocker as a collision and caused the vehicle to stop despite a
    // valid lane change.
    if (target_lateral > config_.lane_width * 0.5 &&
        std::abs(obstacle_lateral) < config_.lane_width * 0.45 &&
        forward_s > config_.change_length + 2.0)
      continue;
    for (const auto& point : result.trajectory.points) {
      const double required_clearance =
          obstacle.radius + config_.obstacle_margin + config_.vehicle_radius;
      if (std::hypot(point.pose.x - obstacle.pose.x, point.pose.y - obstacle.pose.y) <
          required_clearance) {
        result.trajectory = {};
        result.success = false;
        std::ostringstream reason;
        reason << "ring_lane_collision_predicted obstacle=" << obstacle.id << " point=("
               << point.pose.x << "," << point.pose.y << ") clearance=" << required_clearance;
        result.reason = reason.str();
        return result;
      }
    }
  }
  result.trajectory.valid = true;
  result.success = true;
  result.reason = "ok";
  return result;
}

}  // namespace sdc::planning
