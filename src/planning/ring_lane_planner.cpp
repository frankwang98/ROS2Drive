#include "planning/ring_lane_planner.hpp"

#include <algorithm>
#include <cmath>
#include <sstream>
#include <stdexcept>

namespace sdc::planning {
namespace {
constexpr double kPi = 3.14159265358979323846;

double wrap_positive(double angle) {
  angle = std::fmod(angle, 2.0 * kPi);
  return angle < 0.0 ? angle + 2.0 * kPi : angle;
}

bool finite_pose(const domain::Pose2D& pose) {
  return std::isfinite(pose.x) && std::isfinite(pose.y) && std::isfinite(pose.yaw);
}

// Quintic d(s), with d'(0) from the measured heading, d''(0)=0,
// and d(L)=target, d'(L)=d''(L)=0. Derivatives are with respect to
// reference-line arc length, not time or the offset-circle distance.
struct LateralCurve {
  double d0, slope, length, a3, a4, a5, target;

  LateralCurve(double initial, double initial_slope, double goal, double l)
      : d0(initial), slope(initial_slope), length(l), target(goal) {
    const double delta = target - d0;
    const double v = slope * length;
    a3 = 10.0 * delta - 6.0 * v;
    a4 = -15.0 * delta + 8.0 * v;
    a5 = 6.0 * delta - 3.0 * v;
  }

  void sample(double s, double& d, double& dp, double& ddp) const {
    if (s >= length) {
      d = target;
      dp = ddp = 0.0;
      return;
    }
    const double t = s / length;
    d = d0 + slope * s + t * t * t * (a3 + t * (a4 + t * a5));
    dp = slope + t * t * (3.0 * a3 + t * (4.0 * a4 + t * 5.0 * a5)) / length;
    ddp = t * (6.0 * a3 + t * (12.0 * a4 + t * 20.0 * a5)) / (length * length);
  }
};

double segment_distance(const domain::Pose2D& a, const domain::Pose2D& b,
                        const domain::Pose2D& obstacle) {
  const double dx = b.x - a.x, dy = b.y - a.y;
  const double norm_sq = dx * dx + dy * dy;
  const double t = norm_sq > 1e-12
                       ? std::clamp(((obstacle.x - a.x) * dx + (obstacle.y - a.y) * dy) /
                                        norm_sq, 0.0, 1.0)
                       : 0.0;
  return std::hypot(obstacle.x - a.x - t * dx, obstacle.y - a.y - t * dy);
}
}  // namespace

RingLanePlanner::RingLanePlanner() : RingLanePlanner(Config{}) {}
RingLanePlanner::RingLanePlanner(Config config) : config_(config) {
  if (!std::isfinite(config.lane_width) || config.lane_width <= 0.0 ||
      !std::isfinite(config.spacing) || config.spacing < 0.01 || config.spacing > 1.0 ||
      !std::isfinite(config.horizon) || config.horizon <= 0.0 || config.horizon > 1000.0 ||
      !std::isfinite(config.change_length) || config.change_length <= 0.0 ||
      config.change_length > config.horizon ||
      !std::isfinite(config.obstacle_margin) || config.obstacle_margin < 0.0 ||
      !std::isfinite(config.vehicle_radius) || config.vehicle_radius <= 0.0 ||
      config.vehicle_radius >= config.lane_width / 2.0 ||
      !std::isfinite(config.maximum_curvature) || config.maximum_curvature <= 0.0 ||
      !std::isfinite(config.target_d) ||
      (config.target_d != -1.0 && (config.target_d < 0.0 || config.target_d > config.lane_width)))
    throw std::invalid_argument("invalid ring Frenet planner configuration");
}

bool RingLanePlanner::setTargetD(double target_d) {
  if (!std::isfinite(target_d) ||
      (target_d != -1.0 && (target_d < 0.0 || target_d > config_.lane_width)))
    return false;
  // Returning to auto starts a fresh lane decision based on the actual pose.
  if (target_d == -1.0 && config_.target_d != -1.0) {
    maneuver_ = Maneuver::kKeepLeft;
    passed_left_obstacle_ = false;
  }
  config_.target_d = target_d;
  return true;
}

PlanningResult RingLanePlanner::plan(const PlanningInput& input) {
  PlanningResult result;
  if (!input.vehicle.localized || !finite_pose(input.vehicle.pose) ||
      input.reference_path.size() < 3 || !std::isfinite(input.speed_limit) ||
      input.speed_limit < 0.0 || !std::isfinite(input.now_s)) {
    result.reason = "ring_lane_input_invalid";
    return result;
  }

  double reference_radius = 0.0;
  for (const auto& pose : input.reference_path) {
    if (!finite_pose(pose)) {
      result.reason = "ring_lane_reference_invalid";
      return result;
    }
    reference_radius += std::hypot(pose.x, pose.y);
  }
  reference_radius /= static_cast<double>(input.reference_path.size());
  if (!std::isfinite(reference_radius) || reference_radius <= config_.lane_width ||
      config_.horizon >= kPi * reference_radius) {
    result.reason = "ring_lane_reference_invalid";
    return result;
  }
  // This planner implements an origin-centred CCW circle, not arbitrary routes.
  for (const auto& pose : input.reference_path) {
    if (std::abs(std::hypot(pose.x, pose.y) - reference_radius) > 0.05) {
      result.reason = "ring_lane_reference_not_circular";
      return result;
    }
  }
  const double vehicle_radius = std::hypot(input.vehicle.pose.x, input.vehicle.pose.y);
  const double vehicle_angle = std::atan2(input.vehicle.pose.y, input.vehicle.pose.x);
  const double current_d = vehicle_radius - reference_radius;
  const double heading_error = std::atan2(
      std::sin(input.vehicle.pose.yaw - vehicle_angle - kPi / 2.0),
      std::cos(input.vehicle.pose.yaw - vehicle_angle - kPi / 2.0));
  if (std::abs(heading_error) >= 1.2) {
    result.reason = "ring_lane_heading_invalid";
    return result;
  }

  bool left_blocked_ahead = false, right_blocked_ahead = false;
  bool left_obstacle_cleared_behind = false;
  for (const auto& obstacle : input.obstacles) {
    if (!finite_pose(obstacle.pose) || !std::isfinite(obstacle.radius) || obstacle.radius < 0.0) {
      result.reason = "ring_lane_obstacle_invalid";
      return result;
    }
    const double d = std::hypot(obstacle.pose.x, obstacle.pose.y) - reference_radius;
    const double delta = std::atan2(obstacle.pose.y, obstacle.pose.x) - vehicle_angle;
    const double signed_s = std::atan2(std::sin(delta), std::cos(delta)) * reference_radius;
    const double forward_s = wrap_positive(delta) * reference_radius;
    // Include the inflated obstacle beyond the horizon endpoint; otherwise
    // a return manoeuvre can collide before its centre enters the horizon.
    const double clearance = obstacle.radius + config_.obstacle_margin + config_.vehicle_radius;
    const bool ahead = forward_s > 0.5 &&
                       forward_s < config_.horizon + clearance + 2.0 * config_.spacing;
    const bool in_left = std::abs(d) < config_.lane_width * 0.45;
    const bool in_right = std::abs(d - config_.lane_width) < config_.lane_width * 0.45;
    left_blocked_ahead = left_blocked_ahead || (in_left && ahead);
    right_blocked_ahead = right_blocked_ahead || (in_right && ahead);
    left_obstacle_cleared_behind = left_obstacle_cleared_behind || (in_left && signed_s < -6.0);
  }

  if (maneuver_ == Maneuver::kKeepLeft && left_blocked_ahead && !right_blocked_ahead)
    maneuver_ = Maneuver::kChangeRight;
  if (maneuver_ == Maneuver::kChangeLeft && left_blocked_ahead && !right_blocked_ahead)
    maneuver_ = Maneuver::kChangeRight;
  if (maneuver_ == Maneuver::kChangeRight && current_d >= config_.lane_width * 0.85) {
    maneuver_ = Maneuver::kKeepRight;
    passed_left_obstacle_ = false;
  }
  if (maneuver_ == Maneuver::kKeepRight && left_obstacle_cleared_behind)
    passed_left_obstacle_ = true;
  if (maneuver_ == Maneuver::kKeepRight && passed_left_obstacle_ && !left_blocked_ahead &&
      !right_blocked_ahead)
    maneuver_ = Maneuver::kChangeLeft;
  if (maneuver_ == Maneuver::kChangeLeft && current_d <= config_.lane_width * 0.15)
    maneuver_ = Maneuver::kKeepLeft;

  const double policy_d =
      maneuver_ == Maneuver::kChangeRight || maneuver_ == Maneuver::kKeepRight
          ? config_.lane_width : 0.0;
  const double target_d = config_.target_d >= 0.0 ? config_.target_d : policy_d;
  // x'(s)=d' e_r + (1+d/R) e_theta. The outward-positive convention
  // gives d' = -(1+d/R)*tan(yaw-reference_yaw).
  const double initial_slope = -(vehicle_radius / reference_radius) * std::tan(heading_error);
  const LateralCurve curve(current_d, initial_slope, target_d, config_.change_length);
  const int samples = std::max(2, static_cast<int>(std::ceil(config_.horizon / config_.spacing)));
  result.trajectory.frame_id = "map";
  result.trajectory.stamp_s = input.now_s;
  double arc_length = 0.0;
  for (int i = 0; i <= samples; ++i) {
    const double s = config_.horizon * static_cast<double>(i) / samples;
    double d, dp, ddp;
    curve.sample(s, d, dp, ddp);
    const double rho = reference_radius + d;
    const double theta = vehicle_angle + s / reference_radius;
    const double a = rho / reference_radius;
    const double norm_sq = dp * dp + a * a;
    domain::Waypoint point;
    point.pose.x = rho * std::cos(theta);
    point.pose.y = rho * std::sin(theta);
    point.pose.yaw = std::atan2(dp * std::sin(theta) + a * std::cos(theta),
                                dp * std::cos(theta) - a * std::sin(theta));
    point.curvature = (2.0 * dp * dp / reference_radius +
                        a * (rho / (reference_radius * reference_radius) - ddp)) /
                       std::pow(norm_sq, 1.5);
    const double min_d = -config_.lane_width / 2.0 + config_.vehicle_radius;
    const double max_d = 1.5 * config_.lane_width - config_.vehicle_radius;
    if (!finite_pose(point.pose) || !std::isfinite(point.curvature) ||
        d < min_d || d > max_d || std::abs(point.curvature) > config_.maximum_curvature) {
      result.trajectory = {};
      result.reason = "ring_lane_frenet_infeasible";
      return result;
    }
    if (!result.trajectory.points.empty()) {
      const auto& previous = result.trajectory.points.back().pose;
      arc_length += std::hypot(point.pose.x - previous.x, point.pose.y - previous.y);
    }
    point.velocity = input.speed_limit;
    point.relative_time = arc_length / std::max(0.1, input.speed_limit);
    result.trajectory.points.push_back(point);
  }

  // Check the entire sampled polyline, including between samples and near
  // the start. No lane/angle shortcut can waive an actual collision.
  for (const auto& obstacle : input.obstacles) {
    const double clearance = obstacle.radius + config_.obstacle_margin + config_.vehicle_radius;
    for (std::size_t i = 1; i < result.trajectory.points.size(); ++i) {
      const auto& a = result.trajectory.points[i - 1].pose;
      const auto& b = result.trajectory.points[i].pose;
      // Extra clearance for the curved path between nearby sampled points.
      const double chord_sq = (b.x - a.x) * (b.x - a.x) + (b.y - a.y) * (b.y - a.y);
      const double sampling_margin = config_.maximum_curvature * chord_sq / 4.0;
      if (segment_distance(a, b, obstacle.pose) < clearance + sampling_margin) {
        result.trajectory = {};
        std::ostringstream reason;
        reason << "ring_lane_collision_predicted obstacle=" << obstacle.id
               << " clearance=" << clearance;
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
