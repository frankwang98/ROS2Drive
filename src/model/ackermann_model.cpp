#include "model/ackermann_model.hpp"

namespace sdc {

AckermannModel::AckermannModel(const Params& p) : p_(p) {}

void AckermannModel::reset(double x, double y, double yaw, double speed, double steer) {
  x_ = x;
  y_ = y;
  yaw_ = yaw;
  speed_ = speed;
  steer_ = steer;
}

void AckermannModel::update(double speed_cmd, double steer_cmd, double dt) {
  // 1. 前轮转角：目标转角限幅（受最大转角约束）后按最大转角变化率平滑逼近，
  //    避免转向突变造成横向急摆（画龙）。
  steer_cmd = clamp(steer_cmd, -p_.max_steer, p_.max_steer);
  double max_delta = p_.max_steer_rate * dt;
  double ds = steer_cmd - steer_;
  if (std::fabs(ds) > max_delta) {
    steer_ += (ds > 0.0 ? max_delta : -max_delta);
  } else {
    steer_ = steer_cmd;
  }

  // 2. 车速：简化为一阶逼近（可选），此处直接用指令速度作为当前车速，
  //    速度的平滑由上层速度控制器（PID 等）保证。
  speed_ = speed_cmd;

  // 3. 单车运动学积分（一阶前向欧拉）。
  if (dt > 0.0) {
    x_   += speed_ * std::cos(yaw_) * dt;
    y_   += speed_ * std::sin(yaw_) * dt;
    yaw_ += (speed_ / p_.wheelbase) * std::tan(steer_) * dt;
  }
}

}  // namespace sdc
