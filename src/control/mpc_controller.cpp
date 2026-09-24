#include "control/mpc_controller.hpp"

#include <algorithm>
#include <cmath>

namespace sdc {

namespace {
double norm_angle(double a) {
  while (a > M_PI)
    a -= 2.0 * M_PI;
  while (a < -M_PI)
    a += 2.0 * M_PI;
  return a;
}
}  // namespace

MpcController::MpcController() : p_(Params()) {}

MpcController::MpcController(const Params& p) : p_(p) {}

void MpcController::rollout(const State& s0,
                            double speed,
                            const std::vector<double>& u,
                            double dt,
                            std::vector<State>& traj) const {
  int N = p_.horizon;
  double L = p_.wheelbase > 1e-6 ? p_.wheelbase : 2.0;
  traj.clear();
  traj.reserve(N + 1);
  State s = s0;
  traj.push_back(s);
  for (int k = 0; k < N; ++k) {
    double steer = u[static_cast<size_t>(k)];
    double v = speed;
    s.yaw += (v / L) * std::tan(steer) * dt;
    s.x += v * std::cos(s.yaw) * dt;
    s.y += v * std::sin(s.yaw) * dt;
    traj.push_back(s);
  }
}

double MpcController::cost(
    const State& s0, double speed, const std::vector<double>& u, double dt, double goal_yaw) const {
  int N = p_.horizon;
  std::vector<State> traj;
  rollout(s0, speed, u, dt, traj);

  // 参考线：过 s0、方向为 goal_yaw 的直线
  double rx = std::cos(goal_yaw), ry = std::sin(goal_yaw);
  double c = 0.0;
  for (int k = 0; k < N; ++k) {
    const State& s = traj[static_cast<size_t>(k + 1)];
    // 航向误差
    double h_err = norm_angle(goal_yaw - s.yaw);
    // 横向误差：点到参考线的有向距离（参考线右侧为负）
    double dx = s.x - s0.x, dy = s.y - s0.y;
    double l_err = -rx * dy + ry * dx;  // 左正
    c += p_.q_heading * h_err * h_err + p_.q_lateral * l_err * l_err;
    c += p_.r_steer * u[static_cast<size_t>(k)] * u[static_cast<size_t>(k)];
    if (k > 0) {
      double du = u[static_cast<size_t>(k)] - u[static_cast<size_t>(k - 1)];
      c += p_.r_dsteer * du * du;
    }
  }
  return c;
}

double MpcController::compute(double car_x,
                              double car_y,
                              double car_yaw,
                              double goal_x,
                              double goal_y,
                              double speed,
                              double dt) {
  int N = p_.horizon;
  double dt_pred = p_.dt;

  State s0{car_x, car_y, car_yaw};
  double goal_yaw = std::atan2(goal_y - car_y, goal_x - car_x);

  // 初始控制序列：上一时刻输出（保持平滑），全部置为最近转向
  std::vector<double> u(static_cast<size_t>(N), last_steer_);

  // 投影梯度下降求解最优控制序列
  double max_d_steer = p_.max_steer_rate * dt_pred;  // 相邻步允许的最大转角变化
  for (int it = 0; it < p_.iterations; ++it) {
    std::vector<double> grad(static_cast<size_t>(N), 0.0);
    const double eps = 1e-4;
    for (int k = 0; k < N; ++k) {
      double orig = u[static_cast<size_t>(k)];
      u[static_cast<size_t>(k)] = orig + eps;
      double c_plus = cost(s0, speed, u, dt_pred, goal_yaw);
      u[static_cast<size_t>(k)] = orig - eps;
      double c_minus = cost(s0, speed, u, dt_pred, goal_yaw);
      u[static_cast<size_t>(k)] = orig;
      grad[static_cast<size_t>(k)] = (c_plus - c_minus) / (2.0 * eps);
    }
    for (int k = 0; k < N; ++k) {
      u[static_cast<size_t>(k)] -= p_.step_size * grad[static_cast<size_t>(k)];
      // 投影回可行域（转向角 + 变化率约束）
      u[static_cast<size_t>(k)] = clamp(u[static_cast<size_t>(k)], -p_.max_steer, p_.max_steer);
      if (k > 0) {
        double lo = clamp(u[static_cast<size_t>(k - 1)] - max_d_steer, -p_.max_steer, p_.max_steer);
        double hi = clamp(u[static_cast<size_t>(k - 1)] + max_d_steer, -p_.max_steer, p_.max_steer);
        u[static_cast<size_t>(k)] = clamp(u[static_cast<size_t>(k)], lo, hi);
      }
    }
  }

  // 取最优序列首个控制量作为输出（含转向变化率限幅）
  double steer = u[0];
  double max_delta = p_.max_steer_rate * dt;
  if (dt > 1e-6) {
    double delta = clamp(steer - last_steer_, -max_delta, max_delta);
    steer = clamp(last_steer_ + delta, -p_.max_steer, p_.max_steer);
  }
  last_steer_ = steer;
  return steer;
}

}  // namespace sdc
