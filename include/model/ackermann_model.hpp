#ifndef SELF_DRIVING_CAR_MODEL_ACKERMANN_MODEL_HPP
#define SELF_DRIVING_CAR_MODEL_ACKERMANN_MODEL_HPP

#include <cmath>

namespace sdc {

/// 基于阿克曼转向的车辆运动学模型（单车 / bicycle model）。
///
/// 相比直接对横向偏移做"硬赋值"的简化方式，阿克曼模型满足非完整约束：
///   车体只能沿当前朝向方向前进，转向通过前轮转角 delta 控制，转弯半径
///   由  L / tan(delta) 决定，因而车头朝向与行驶方向始终一致，不会出现
///   随意斜线滑移（侧滑）或横向急摆的"画龙"现象。
///
/// 状态：x, y（世界坐标，米）、yaw（朝向角，弧度）、speed（车速，m/s）、
///       steer（前轮转角，弧度）。
/// 微分方程（连续单车模型，dt 内用一阶前向欧拉积分）：
///   dx/dt   = v * cos(yaw)
///   dy/dt   = v * sin(yaw)
///   dyaw/dt = (v / wheelbase) * tan(steer)
class AckermannModel {
 public:
  struct Params {
    double wheelbase = 2.0;       // 轴距（米）
    double max_steer = 0.55;      // 前轮最大转角（弧度，约 ±31.5°）
    double max_steer_rate = 0.9;  // 前轮最大转角变化率（rad/s），抑制"画龙"
  };

  explicit AckermannModel();
  explicit AckermannModel(const Params& p);

  /// 重置状态。
  void reset(double x, double y, double yaw, double speed = 0.0, double steer = 0.0);

  /// 推进一个时间步：给定目标车速与目标前轮转角，经限幅后积分运动学。
  /// @param speed_cmd 目标车速（m/s，可为负表示倒车）
  /// @param steer_cmd 目标前轮转角（弧度）
  /// @param dt        时间步长（秒）
  void update(double speed_cmd, double steer_cmd, double dt);

  // 状态访问
  double x() const {
    return x_;
  }
  double y() const {
    return y_;
  }
  double yaw() const {
    return yaw_;
  }
  double speed() const {
    return speed_;
  }
  double steer() const {
    return steer_;
  }
  double wheelbase() const {
    return p_.wheelbase;
  }

 private:
  double clamp(double v, double lo, double hi) const {
    return std::fmin(hi, std::fmax(lo, v));
  }

  Params p_;
  double x_{0.0};
  double y_{0.0};
  double yaw_{0.0};
  double speed_{0.0};
  double steer_{0.0};
};

}  // namespace sdc

#endif  // SELF_DRIVING_CAR_MODEL_ACKERMANN_MODEL_HPP
