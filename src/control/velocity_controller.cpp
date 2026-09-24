#include "control/velocity_controller.hpp"

#include <algorithm>
#include <cmath>

namespace sdc {

// 斜坡控制的固定加减速斜率（与原 MotorController 默认一致）
namespace {
constexpr double kRampAccel = 1.0;  // m/s^2
constexpr double kRampDecel = 3.0;  // m/s^2
}  // namespace

const char* velocity_algorithm_name(VelocityAlgorithm a) {
  switch (a) {
    case VelocityAlgorithm::kPid:
      return "PID";
    case VelocityAlgorithm::kBangBang:
      return "Bang-Bang";
    case VelocityAlgorithm::kRamp:
      return "Ramp";
    default:
      return "Unknown";
  }
}

VelocityController::VelocityController() {}

void VelocityController::reset() {
  pid_.reset();
}

double VelocityController::update(double target_speed, double current_speed, double dt) {
  switch (algo_) {
    case VelocityAlgorithm::kPid: {
      // PID 以加速度为控制量，对当前速度积分得到下一时刻速度
      double accel = pid_.compute(target_speed, current_speed);
      double next = current_speed + accel * dt;
      return std::max(0.0, next);
    }
    case VelocityAlgorithm::kBangBang: {
      // 以最大加速度（或最大减速度）冲向目标速度
      double tol = 0.05;
      if (target_speed > current_speed + tol) {
        return std::min(target_speed, current_speed + 4.0 * dt);
      } else if (target_speed < current_speed - tol) {
        return std::max(target_speed, current_speed - 6.0 * dt);
      }
      return current_speed;
    }
    case VelocityAlgorithm::kRamp:
    default: {
      // 斜坡控制：按固定加减速斜率逼近目标速度
      double rate = (target_speed >= current_speed) ? kRampAccel : kRampDecel;
      double max_delta = rate * dt;
      double next = current_speed;
      if (target_speed > current_speed) {
        next = std::min(target_speed, current_speed + max_delta);
      } else {
        next = std::max(target_speed, current_speed - max_delta);
      }
      return std::max(0.0, next);
    }
  }
}

}  // namespace sdc
