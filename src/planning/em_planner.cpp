#include "planning/em_planner.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace sdc {

EmPlanner::EmPlanner(const EmPlannerParams& p) : p_(p) {}

Vec2 EmPlanner::center_at(double s0_angle, double s) const {
  double theta = s0_angle + s / p_.ring_radius;
  return Vec2{p_.ring_radius * std::cos(theta), p_.ring_radius * std::sin(theta)};
}

// 五阶多项式 d(s) = a0 + a1 s + a2 s² + a3 s³ + a4 s⁴ + a5 s⁵
// 边界条件：
//   d(0)=d0, d'(0)=d0_prime, d''(0)=0
//   d(L)=d_target, d'(L)=0, d''(L)=0
// 经典闭合解（Frenet 采样常用）。
void EmPlanner::build_profile(double d0,
                              double d0_prime,
                              double d_target,
                              double lookahead,
                              int samples,
                              std::vector<double>& d_profile) const {
  double L = std::fmax(lookahead, 1e-6);
  double L2 = L * L, L3 = L2 * L, L4 = L3 * L, L5 = L4 * L;

  double a0 = d0;
  double a1 = d0_prime;
  double a2 = 0.0;
  double a3 = (10.0 * (d_target - d0) - 6.0 * d0_prime * L) / L3;
  double a4 = (-15.0 * (d_target - d0) + 7.0 * d0_prime * L) / L4;
  double a5 = (6.0 * (d_target - d0) - 3.0 * d0_prime * L) / L5;

  d_profile.clear();
  d_profile.reserve(static_cast<size_t>(samples + 1));
  for (int k = 0; k <= samples; ++k) {
    double s = L * static_cast<double>(k) / static_cast<double>(samples);
    double s2 = s * s, s3 = s2 * s, s4 = s3 * s, s5 = s4 * s;
    d_profile.push_back(a0 + a1 * s + a2 * s2 + a3 * s3 + a4 * s4 + a5 * s5);
  }
}

double EmPlanner::evaluate(const LatticeTrajectory& traj,
                           const std::vector<Obstacle>& obstacles) const {
  double cost = 0.0;

  // 障碍物代价
  for (const auto& ob : obstacles) {
    double ob_min = std::numeric_limits<double>::infinity();
    for (const auto& pt : traj.path) {
      double dx = pt.x - ob.position.x;
      double dy = pt.y - ob.position.y;
      double d = std::sqrt(dx * dx + dy * dy) - ob.radius;
      if (d < ob_min)
        ob_min = d;
    }
    if (ob_min <= 0.0) {
      cost += 1e5;  // 穿障：极大代价
    } else if (ob_min < p_.obstacle_safe_dist) {
      cost += p_.obstacle_cost_gain * std::pow(1.0 - ob_min / p_.obstacle_safe_dist, 2);
    }
  }

  // 横向偏移代价
  cost += p_.lateral_cost_gain * std::fabs(traj.lateral_offset);

  // 平滑代价：相邻线段方向夹角（近似曲率能量），路径越直越优
  if (traj.path.size() >= 3) {
    double sum_d2 = 0.0;
    for (size_t i = 1; i + 1 < traj.path.size(); ++i) {
      double dx1 = traj.path[i].x - traj.path[i - 1].x;
      double dy1 = traj.path[i].y - traj.path[i - 1].y;
      double dx2 = traj.path[i + 1].x - traj.path[i].x;
      double dy2 = traj.path[i + 1].y - traj.path[i].y;
      // 相邻线段方向夹角（曲率近似）
      double c = dx1 * dx2 + dy1 * dy2;
      double n1 = std::hypot(dx1, dy1), n2 = std::hypot(dx2, dy2);
      if (n1 > 1e-6 && n2 > 1e-6) {
        double cos_a = c / (n1 * n2);
        cos_a = std::fmax(-1.0, std::fmin(1.0, cos_a));
        double angle = std::acos(cos_a);
        sum_d2 += angle * angle;
      }
    }
    cost += p_.smoothness_gain * sum_d2;
  }
  return cost;
}

void EmPlanner::plan(double pose_x,
                     double pose_y,
                     double yaw,
                     const std::vector<Obstacle>& obstacles,
                     std::vector<LatticeTrajectory>& out) const {
  out.clear();

  // 小车当前在环道中心线上的角度（由位置反算）
  double s0_angle = std::atan2(pose_y, pose_x);
  // 当前横向偏移（车在环道中心线上的偏移，用径向投影计算）
  double theta0 = s0_angle;
  double nx = std::cos(theta0), ny = std::sin(theta0);
  double d0 = (pose_x - p_.ring_radius * nx) * nx + (pose_y - p_.ring_radius * ny) * ny;
  // 环道切线方向角（CCW 环上某点的前进方向 = 径向角 + π/2）
  double tangent_yaw = theta0 + M_PI / 2.0;
  // 初始横向偏移随弧长的导数：dd/ds ≈ tan(车头相对切线方向的夹角)
  double d0_prime = std::tan(yaw - tangent_yaw);

  // 横向候选偏移列表（从负到正）
  std::vector<double> lat_offsets;
  for (double r = -p_.max_lateral; r <= p_.max_lateral + 1e-9; r += p_.lateral_step) {
    lat_offsets.push_back(r);
  }
  if (lat_offsets.empty())
    lat_offsets.push_back(d0);

  double best_cost = std::numeric_limits<double>::infinity();
  int best_idx = -1;

  for (size_t i = 0; i < lat_offsets.size(); ++i) {
    double r_target = lat_offsets[i];

    // E 步：quintic 生成横向偏移剖面（考虑当前横向偏移与车头朝向）
    std::vector<double> d_profile;
    build_profile(d0, d0_prime, r_target, p_.lookahead, p_.s_samples, d_profile);

    LatticeTrajectory traj;
    traj.lateral_offset = r_target;

    // Frenet → Cartesian
    for (int k = 0; k <= p_.s_samples; ++k) {
      double s = p_.lookahead * static_cast<double>(k) / p_.s_samples;
      double d = d_profile[static_cast<size_t>(k)];
      double theta = s0_angle + s / p_.ring_radius;
      Vec2 center{p_.ring_radius * std::cos(theta), p_.ring_radius * std::sin(theta)};
      Vec2 radial{std::cos(theta), std::sin(theta)};
      traj.path.push_back(Vec2{center.x + d * radial.x, center.y + d * radial.y});
    }

    // M 步：评估代价并计算速度 / 可通行性
    traj.cost = evaluate(traj, obstacles);

    double min_d = std::numeric_limits<double>::infinity();
    for (const auto& ob : obstacles) {
      for (const auto& pt : traj.path) {
        double dx = pt.x - ob.position.x;
        double dy = pt.y - ob.position.y;
        double d = std::sqrt(dx * dx + dy * dy) - ob.radius;
        if (d < min_d)
          min_d = d;
      }
    }
    traj.clearance =
        (min_d == std::numeric_limits<double>::infinity()) ? p_.obstacle_safe_dist : min_d;
    traj.blocked = (min_d < 0.35);
    traj.speed = p_.max_speed;
    if (min_d < p_.obstacle_safe_dist && min_d > 0.0) {
      traj.speed = std::max(0.5, p_.max_speed * min_d / p_.obstacle_safe_dist);
    } else if (min_d <= 0.0) {
      traj.speed = 0.5;
    }

    if (traj.cost < best_cost) {
      best_cost = traj.cost;
      best_idx = static_cast<int>(i);
    }
    out.push_back(std::move(traj));
  }

  if (best_idx >= 0 && best_idx < static_cast<int>(out.size())) {
    out[static_cast<size_t>(best_idx)].selected = true;
  }
}

}  // namespace sdc
