#include "control/legacy_controller_adapter.hpp"

#include <algorithm>
#include <cmath>

namespace sdc::control {

LegacyControllerAdapter::LegacyControllerAdapter(Type type, double wheelbase,
                                                 double max_steering,
                                                 double dt_s)
    : type_(type),
      stanley_(SteeringController::Params{1.2, 0.8, 0.5, max_steering}),
      lqr_(LqrController::Params{2.0, 0.5, 3.0, 1.0, dt_s, wheelbase,
                                 max_steering, 0.9, 0.5}),
      mpc_(MpcController::Params{15, dt_s, wheelbase, max_steering, 0.9,
                                 2.0, 3.0, 0.5, 2.0, 40, 0.06}) {}

domain::ControlCommand LegacyControllerAdapter::compute(const ControllerInput& input) {
  domain::ControlCommand command;
  if (!input.trajectory.valid || input.trajectory.points.empty()) {
    command.brake = 1.0;
    return command;
  }
  const auto& target = input.trajectory.points[std::min<std::size_t>(5, input.trajectory.points.size() - 1)];
  const double x = input.vehicle.pose.x;
  const double y = input.vehicle.pose.y;
  const double yaw = input.vehicle.pose.yaw;
  const double speed = input.vehicle.velocity.linear;
  switch (type_) {
    case Type::kStanley:
      command.steering_angle = stanley_.compute(x, y, yaw, target.pose.x, target.pose.y, speed);
      break;
    case Type::kLqr:
      command.steering_angle = lqr_.compute(x, y, yaw, target.pose.x, target.pose.y, speed);
      break;
    case Type::kMpc:
      command.steering_angle = mpc_.compute(x, y, yaw, target.pose.x, target.pose.y, speed,
                                            input.dt_s);
      break;
  }
  command.target_speed = input.trajectory.points.front().velocity;
  return command;
}

void LegacyControllerAdapter::reset() {
  stanley_.reset();
  lqr_.reset();
  mpc_.reset();
}

}  // namespace sdc::control
