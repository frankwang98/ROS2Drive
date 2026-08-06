#include "control/motor_controller.hpp"

#include <algorithm>

namespace sdc {

MotorController::MotorController(double accel, double decel)
    : max_accel_(accel), max_decel_(decel) {}

double MotorController::target_speed(Action action) {
  switch (action) {
    case Action::kAccelerate:
      return 3.0;
    case Action::kCruise:
      return 2.0;
    case Action::kBrake:
      return 0.5;
    case Action::kStop:
      return 0.0;
    default:
      return 0.0;
  }
}

double MotorController::update(Action action, double current_speed, double dt) {
  double target = target_speed(action);
  double rate = (target >= current_speed) ? max_accel_ : max_decel_;
  double max_delta = rate * dt;

  double next = current_speed;
  if (target > current_speed) {
    next = std::min(target, current_speed + max_delta);
  } else {
    next = std::max(target, current_speed - max_delta);
  }
  return std::max(0.0, next);
}

}  // namespace sdc
