#ifndef SELF_DRIVING_CAR_CONTROL_VELOCITY_CONTROLLER_HPP
#define SELF_DRIVING_CAR_CONTROL_VELOCITY_CONTROLLER_HPP

#include <string>

#include "control/pid_controller.hpp"

namespace sdc {

/// 速度控制算法类型。
enum class VelocityAlgorithm {
  kPid,       // PID 闭环控制
  kBangBang,  // Bang-Bang 开关控制（全速接近目标）
  kRamp,      // 基于固定加减速斜率的斜坡控制（原 MotorController 逻辑）
};

const char* velocity_algorithm_name(VelocityAlgorithm a);

/// 速度控制器：根据目标速度与当前速度计算下一时刻速度。
/// 封装多种控制算法，可通过 set_algorithm 切换。
class VelocityController {
 public:
  VelocityController();

  /// 切换控制算法。
  void set_algorithm(VelocityAlgorithm a) {
    algo_ = a;
  }
  VelocityAlgorithm algorithm() const {
    return algo_;
  }

  /// 设置 PID 参数。
  void set_pid_params(const PidController::Params& p) {
    pid_.set_params(p);
  }

  /// 根据目标速度与当前速度计算下一时刻速度（m/s）。
  // Manual driving can opt into signed velocity; autonomous callers retain
  // the existing forward-only PID/ramp behavior by default.
  double update(double target_speed, double current_speed, double dt, bool allow_reverse = false);

  /// 重置内部状态。
  void reset();

 private:
  VelocityAlgorithm algo_{VelocityAlgorithm::kPid};
  PidController pid_;
};

}  // namespace sdc

#endif  // SELF_DRIVING_CAR_CONTROL_VELOCITY_CONTROLLER_HPP
