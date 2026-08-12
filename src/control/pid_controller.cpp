#include "control/pid_controller.hpp"

#include <algorithm>

namespace sdc {

PidController::PidController(const Params& p) : p_(p) {}

void PidController::reset() {
  integral_ = 0.0;
  prev_error_ = 0.0;
  prev_derivative_ = 0.0;
  first_run_ = true;
}

double PidController::compute(double setpoint, double feedback) {
  double error = setpoint - feedback;

  // 积分项（条件积分：只在误差不大时累积，避免积分饱和）
  integral_ += error * p_.dt;
  integral_ = std::max(-p_.integral_limit,
                       std::min(p_.integral_limit, integral_));

  // 微分项
  double derivative = 0.0;
  if (!first_run_) {
    // 一阶低通滤波，抑制高频噪声
    double raw = (error - prev_error_) / p_.dt;
    derivative = prev_derivative_ + 0.2 * (raw - prev_derivative_);
  } else {
    first_run_ = false;
  }

  // PID 输出
  double u = p_.kp * error + p_.ki * integral_ + p_.kd * derivative;

  // 输出限幅
  u = std::max(p_.out_min, std::min(p_.out_max, u));

  prev_error_ = error;
  prev_derivative_ = derivative;
  return u;
}

}  // namespace sdc
