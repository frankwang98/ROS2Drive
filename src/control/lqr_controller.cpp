#include "control/lqr_controller.hpp"

#include <cmath>

namespace sdc {

LqrController::LqrController() : p_(Params()) {}

LqrController::LqrController(const Params& p) : p_(p) {}

void LqrController::solve_gains(double v, double K[3]) const {
  double L = p_.wheelbase;
  if (L < 1e-6)
    L = 2.0;
  double dt = p_.dt;
  double vv = std::fmax(std::fabs(v), p_.min_v);

  // 离散系统（一阶前向欧拉）：
  //   x[k+1] = A_d x[k] + B_d u[k]
  double A_d[3][3] = {
      {1.0, dt, 0.0},
      {0.0, 1.0, vv * dt},
      {0.0, 0.0, 1.0},
  };
  // B_d = [0, vv²/L * dt, vv/L * dt]^T
  double B_d[3] = {0.0, vv * vv / L * dt, vv / L * dt};

  // 权重矩阵
  double Q[3][3] = {{0.0}};
  Q[0][0] = p_.q_e;
  Q[1][1] = p_.q_e_dot;
  Q[2][2] = p_.q_d_psi;
  double R = p_.r_steer;
  if (R < 1e-9)
    R = 1e-9;

  // 离散代数黎卡提方程迭代求解：P = Q + A^T P A - A^T P B (R + B^T P B)^-1 B^T P A
  double P[3][3] = {{0.0}};
  for (int iter = 0; iter < 200; ++iter) {
    // A^T P
    double AtP[3][3];
    for (int i = 0; i < 3; ++i)
      for (int j = 0; j < 3; ++j) {
        AtP[i][j] = 0.0;
        for (int k = 0; k < 3; ++k)
          AtP[i][j] += A_d[k][i] * P[k][j];
      }
    // B^T P
    double BtP[3];
    for (int j = 0; j < 3; ++j) {
      BtP[j] = 0.0;
      for (int k = 0; k < 3; ++k)
        BtP[j] += B_d[k] * P[k][j];
    }
    // B^T P B  (标量)
    double BtPB = 0.0;
    for (int k = 0; k < 3; ++k)
      BtPB += BtP[k] * B_d[k];
    double denom = R + BtPB;

    // (R + B^T P B)^-1 * B^T P A   —— 即增益分子，这里只用于迭代
    // 先算 P A
    double PA[3][3];
    for (int i = 0; i < 3; ++i)
      for (int j = 0; j < 3; ++j) {
        PA[i][j] = 0.0;
        for (int k = 0; k < 3; ++k)
          PA[i][j] += P[i][k] * A_d[k][j];
      }
    // B^T P A (1x3)
    double BtPA[3];
    for (int j = 0; j < 3; ++j) {
      BtPA[j] = 0.0;
      for (int k = 0; k < 3; ++k)
        BtPA[j] += B_d[k] * PA[k][j];
    }

    // P_new = Q + A^T P A - (B^T P A)^T (B^T P A) / (R + B^T P B)
    double Pnew[3][3];
    for (int i = 0; i < 3; ++i)
      for (int j = 0; j < 3; ++j) {
        double AtPA = 0.0;
        for (int k = 0; k < 3; ++k)
          AtPA += AtP[i][k] * A_d[k][j];
        Pnew[i][j] = Q[i][j] + AtPA - (BtPA[i] * BtPA[j]) / denom;
      }

    double diff = 0.0;
    for (int i = 0; i < 3; ++i)
      for (int j = 0; j < 3; ++j)
        diff += std::fabs(Pnew[i][j] - P[i][j]);
    for (int i = 0; i < 3; ++i)
      for (int j = 0; j < 3; ++j)
        P[i][j] = Pnew[i][j];
    if (diff < 1e-6)
      break;
  }

  // 反馈增益 K = (R + B^T P B)^-1 B^T P A
  double BtP[3];
  for (int j = 0; j < 3; ++j) {
    BtP[j] = 0.0;
    for (int k = 0; k < 3; ++k)
      BtP[j] += B_d[k] * P[k][j];
  }
  double BtPB = 0.0;
  for (int k = 0; k < 3; ++k)
    BtPB += BtP[k] * B_d[k];
  double denom = R + BtPB;

  double PA[3][3];
  for (int i = 0; i < 3; ++i)
    for (int j = 0; j < 3; ++j) {
      PA[i][j] = 0.0;
      for (int k = 0; k < 3; ++k)
        PA[i][j] += P[i][k] * A_d[k][j];
    }
  for (int j = 0; j < 3; ++j) {
    double btpa = 0.0;
    for (int k = 0; k < 3; ++k)
      btpa += B_d[k] * PA[k][j];
    K[j] = btpa / denom;
  }
}

double LqrController::compute(
    double car_x, double car_y, double car_yaw, double goal_x, double goal_y, double speed) {
  double dx = goal_x - car_x;
  double dy = goal_y - car_y;
  double d = std::hypot(dx, dy);
  if (d < 1e-6)
    return 0.0;

  // 航向角误差
  double goal_yaw = std::atan2(dy, dx);
  double d_psi = goal_yaw - car_yaw;
  while (d_psi > M_PI)
    d_psi -= 2.0 * M_PI;
  while (d_psi < -M_PI)
    d_psi += 2.0 * M_PI;

  // 横向误差（车体坐标系 y，左正）
  double c = std::cos(car_yaw), s = std::sin(car_yaw);
  double e = -s * dx + c * dy;
  // 横向误差变化率 ≈ v * sin(d_psi)
  double vv = std::fmax(std::fabs(speed), p_.min_v);
  double e_dot = vv * std::sin(d_psi);

  // 求解最优增益
  double K[3] = {0.0, 0.0, 0.0};
  solve_gains(vv, K);

  // 控制量 u = -K·x，对应 tan(δ)，反解 δ。
  // 模型 d_psi' = (v/L)·u 中 u 为「转向」方向，而物理上 d_psi' = -(v/L)·tan(δ)，
  // 故 δ = atan(-u)。
  double x[3] = {e, e_dot, d_psi};
  double u = 0.0;
  for (int i = 0; i < 3; ++i)
    u -= K[i] * x[i];
  double steer = std::atan(clamp(-u, -p_.max_steer, p_.max_steer));

  // 转向变化率限幅，抑制"画龙"
  double max_delta = p_.max_steer_rate * p_.dt;
  double delta = clamp(steer - last_steer_, -max_delta, max_delta);
  steer = clamp(last_steer_ + delta, -p_.max_steer, p_.max_steer);
  last_steer_ = steer;
  return steer;
}

}  // namespace sdc
