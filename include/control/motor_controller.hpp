#ifndef SELF_DRIVING_CAR_CONTROL_MOTOR_CONTROLLER_HPP
#define SELF_DRIVING_CAR_CONTROL_MOTOR_CONTROLLER_HPP

#include "legacy/decision/decision_maker.hpp"

namespace sdc {

/**
 * 电机控制器：根据决策行为，将当前速度平滑调整到目标速度。
 * 目标速度：
 *   - 加速: 3.0 m/s
 *   - 匀速: 2.0 m/s
 *   - 减速: 0.5 m/s
 *   - 停车: 0.0 m/s
 */
class MotorController {
 public:
  explicit MotorController(double accel = 1.0, double decel = 3.0);

  /// 根据行为计算并更新当前速度（m/s），返回更新后的速度。
  double update(Action action, double current_speed, double dt);

 private:
  double max_accel_;
  double max_decel_;

  static double target_speed(Action action);
};

}  // namespace sdc

#endif  // SELF_DRIVING_CAR_CONTROL_MOTOR_CONTROLLER_HPP
