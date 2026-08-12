#ifndef SELF_DRIVING_CAR_CONTROL_PID_CONTROLLER_HPP
#define SELF_DRIVING_CAR_CONTROL_PID_CONTROLLER_HPP

namespace sdc {

/**
 * PID 控制器（离散时间形式，位置式 PID）。
 *
 *   u(t) = Kp*e(t) + Ki*Σ(e*dt) + Kd*(e(t)-e(t-1))/dt
 *
 * 用于速度 / 角速度等一维被控量的闭环控制。
 * 带输出限幅、积分抗饱和（条件积分）与微分低通滤波。
 */
class PidController {
 public:
  struct Params {
    double kp = 2.0;     // 比例系数
    double ki = 0.5;     // 积分系数
    double kd = 0.1;     // 微分系数
    double out_min = -4.0;  // 输出下限（如最大减速度）
    double out_max =  4.0;  // 输出上限（如最大加速度）
    double dt = 0.05;       // 默认采样周期（秒）
    double integral_limit = 20.0;  // 积分项限幅（抗饱和）
  };

  explicit PidController(const Params& p = Params());

  /// 重置内部状态（误差积分 / 上次误差 / 上次微分）。
  void reset();

  /// 根据设定值与当前值计算控制量。
  double compute(double setpoint, double feedback);

  /// 更新/替换参数。
  void set_params(const Params& p) { p_ = p; }

 private:
  Params p_;
  double integral_{0.0};
  double prev_error_{0.0};
  double prev_derivative_{0.0};
  bool   first_run_{true};
};

}  // namespace sdc

#endif  // SELF_DRIVING_CAR_CONTROL_PID_CONTROLLER_HPP
