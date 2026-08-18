#ifndef SELF_DRIVING_CAR_CONTROL_LQR_CONTROLLER_HPP
#define SELF_DRIVING_CAR_CONTROL_LQR_CONTROLLER_HPP

namespace sdc {

/// 基于线性化车辆跟踪误差模型的 LQR（线性二次型调节器）横向控制器。
///
/// 以单车运动学模型为基础，在 Frenet/车体坐标系下建立「横向误差跟踪」的
/// 线性状态空间模型，通过离散代数黎卡提方程（DARE）迭代求解最优反馈增益 K，
/// 再由 u = -K·x 给出前轮转角。相比纯 Stanley 的启发式增益，LQR 在给定
/// Q/R 权重下在「响应速度」与「控制能量」之间给出最优折中。
///
/// 状态向量 x = [e, e_dot, d_psi]^T：
///   e      横向误差（米，左正）
///   e_dot  横向误差变化率（m/s）
///   d_psi  航向角误差（弧度）
/// 连续模型（设 u = tan(δ)）：
///   e'      = e_dot
///   e_dot'  = v * d_psi_dot = (v² / L) * u
///   d_psi'  = (v / L) * u
///
/// 通过一阶前向欧拉离散后，迭代求解离散代数黎卡提方程得到最优增益 K，
/// 输出前轮转角 δ = atan(-K·x)，并进行限幅。
class LqrController {
 public:
  struct Params {
    double q_e      = 2.0;   // 横向误差权重
    double q_e_dot  = 0.5;   // 横向误差变化率权重
    double q_d_psi  = 3.0;   // 航向角误差权重
    double r_steer  = 1.0;   // 转向控制量权重
    double dt       = 0.05;  // 离散化步长（秒）
    double wheelbase = 2.0;  // 轴距（米）
    double max_steer  = 0.55; // 输出最大前轮转角（弧度）
    double max_steer_rate = 0.9; // 前轮最大转角变化率（rad/s），抑制"画龙"
    double min_v     = 0.5;  // 防除零的最小车速（m/s）
  };

  explicit LqrController();
  explicit LqrController(const Params& p);

  /// 计算目标前轮转角（弧度）。
  /// @param car_x, car_y, car_yaw 小车当前位姿
  /// @param goal_x, goal_y        期望路径上的目标点（世界坐标）
  /// @param speed                 当前车速（m/s）
  /// @return 前轮转角（弧度，正值左转）
  double compute(double car_x, double car_y, double car_yaw,
                 double goal_x, double goal_y, double speed);

  /// 重置内部状态（限幅缓存的转角）。
  void reset() { last_steer_ = 0.0; }

 private:
  /// 迭代求解离散代数黎卡提方程，返回反馈增益 K[3]（x = [e, e_dot, d_psi]）。
  void solve_gains(double v, double K[3]) const;
  double clamp(double v, double lo, double hi) const {
    return v < lo ? lo : (v > hi ? hi : v);
  }

  Params p_;
  double last_steer_{0.0};   // 上一时刻输出转角，用于限速
};

}  // namespace sdc

#endif  // SELF_DRIVING_CAR_CONTROL_LQR_CONTROLLER_HPP
