#ifndef SELF_DRIVING_CAR_CONTROL_MPC_CONTROLLER_HPP
#define SELF_DRIVING_CAR_CONTROL_MPC_CONTROLLER_HPP

#include <vector>

namespace sdc {

/// 基于模型预测控制（MPC）的横向/转向控制器。
///
/// 在每个控制周期，以当前小车位姿为初值，用单车运动学模型向前滚动预测
/// 未来 N 步轨迹，并对转向指令序列做有限时域最优化（滚动时域控制）：
///   代价 = Σ(航向误差² + 横向误差²) + Σ(转向能量) + Σ(转向变化率)
/// 采用**投影梯度下降**求解（将每个转向角投影回可行域），取最优序列的
/// 首个转向角作为当前输出。相比 Stanley / LQR 的单步反馈，MPC 显式地
/// 考虑未来时域内的运动约束，更适合高速 / 大曲率场景。
///
/// 纯 C++ 实现，不依赖 Eigen 等外部数值库。
class MpcController {
 public:
  struct Params {
    int    horizon     = 15;    // 预测/控制时域长度 N
    double dt          = 0.05;  // 预测步长（秒）
    double wheelbase   = 2.0;   // 轴距（米）
    double max_steer   = 0.55;  // 前轮最大转角（弧度）
    double max_steer_rate = 0.9; // 相邻步最大转向变化率（rad/s·dt）
    double q_heading   = 2.0;   // 航向误差权重
    double q_lateral   = 3.0;   // 横向误差权重
    double r_steer     = 0.5;   // 转向能量权重
    double r_dsteer    = 2.0;   // 转向变化率权重（平滑）
    int    iterations  = 40;    // 梯度下降迭代次数
    double step_size   = 0.06;  // 梯度下降学习率
  };

  explicit MpcController();
  explicit MpcController(const Params& p);

  /// 计算目标前轮转角（弧度）。
  /// @param car_x, car_y, car_yaw 小车当前位姿
  /// @param goal_x, goal_y        期望目标点（世界坐标）
  /// @param speed                 当前车速（m/s）
  /// @param dt                    调用周期步长（秒）
  /// @return 前轮转角（弧度，正值左转）
  double compute(double car_x, double car_y, double car_yaw,
                 double goal_x, double goal_y, double speed, double dt);

  void reset() { last_steer_ = 0.0; }

 private:
  struct State { double x = 0.0, y = 0.0, yaw = 0.0; };

  /// 用单车模型从 state 出发，给定转向序列 u，滚动预测 N 步，输出轨迹。
  void rollout(const State& s0, double speed, const std::vector<double>& u,
               double dt, std::vector<State>& traj) const;

  /// 计算目标函数值（航向/横向误差 + 控制能量 + 控制变化率）。
  double cost(const State& s0, double speed, const std::vector<double>& u,
              double dt, double goal_yaw) const;

  double clamp(double v, double lo, double hi) const {
    return v < lo ? lo : (v > hi ? hi : v);
  }

  Params p_;
  double last_steer_{0.0};
};

}  // namespace sdc

#endif  // SELF_DRIVING_CAR_CONTROL_MPC_CONTROLLER_HPP
