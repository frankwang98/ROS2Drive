#include "control/trajectory_controller.hpp"

#include <algorithm>
#include <cmath>

namespace sdc::control {

PurePursuitController::PurePursuitController() = default;
PurePursuitController::PurePursuitController(Config config) : config_(config) {}

domain::ControlCommand PurePursuitController::compute(const ControllerInput& input) {
  domain::ControlCommand command;
  if (!input.trajectory.valid || input.trajectory.points.empty()) {
    command.brake = 1.0;
    return command;
  }

  const double desired_lookahead = std::max(
      config_.minimum_lookahead, std::abs(input.vehicle.velocity.linear) * config_.lookahead_time);
  const domain::Waypoint* target = &input.trajectory.points.back();
  for (const auto& point : input.trajectory.points) {
    const double distance =
        std::hypot(point.pose.x - input.vehicle.pose.x, point.pose.y - input.vehicle.pose.y);
    if (distance >= desired_lookahead) {
      target = &point;
      break;
    }
  }

  const double dx = target->pose.x - input.vehicle.pose.x;
  const double dy = target->pose.y - input.vehicle.pose.y;
  const double lookahead = std::max(0.2, std::hypot(dx, dy));
  const double alpha = std::atan2(dy, dx) - input.vehicle.pose.yaw;
  command.steering_angle =
      std::clamp(std::atan2(2.0 * config_.wheelbase * std::sin(alpha), lookahead),
                 -config_.maximum_steering,
                 config_.maximum_steering);
  command.target_speed = input.trajectory.points.front().velocity;
  return command;
}

}  // namespace sdc::control
