#include "planning/lattice_planner.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace sdc {

LatticePlanner::LatticePlanner(const LatticeParams& p) : p_(p) {}

Vec2 LatticePlanner::center_at(double s0_angle, double s) const {
  double theta = s0_angle + s / p_.ring_radius;
  return Vec2{p_.ring_radius * std::cos(theta),
              p_.ring_radius * std::sin(theta)};
}

double LatticePlanner::obstacle_cost(
    const LatticeTrajectory& traj,
    const std::vector<Obstacle>& obstacles) const {
  double cost = 0.0;
  for (const auto& ob : obstacles) {
    double min_d = std::numeric_limits<double>::infinity();
    for (const auto& pt : traj.path) {
      double dx = pt.x - ob.position.x;
      double dy = pt.y - ob.position.y;
      double d = std::sqrt(dx * dx + dy * dy) - ob.radius;
      if (d < min_d) min_d = d;
    }
    if (min_d <= 0.0) {
      // 穿障：极大代价
      cost += 1e5;
    } else if (min_d < p_.obstacle_safe_dist) {
      // 越靠近障碍物代价越大
      cost += p_.obstacle_cost_gain *
              std::pow(1.0 - min_d / p_.obstacle_safe_dist, 2);
    }
  }
  return cost;
}

void LatticePlanner::plan(double pose_x, double pose_y, double yaw,
                          const std::vector<Obstacle>& obstacles,
                          std::vector<LatticeTrajectory>& out) const {
  out.clear();

  // 小车当前在环道中心线上的角度（由位置反算）
  double s0_angle = std::atan2(pose_y, pose_x);
  // 径向（从圆心指向外）方向单位向量
  // n_out = (cos theta0, sin theta0)

  // 横向候选偏移列表（从负到正）
  std::vector<double> lat_offsets;
  for (double r = -p_.max_lateral; r <= p_.max_lateral + 1e-9;
       r += p_.lateral_step) {
    lat_offsets.push_back(r);
  }
  if (lat_offsets.empty()) lat_offsets.push_back(0.0);

  double best_cost = std::numeric_limits<double>::infinity();
  int    best_idx = -1;

  for (size_t i = 0; i < lat_offsets.size(); ++i) {
    double r_target = lat_offsets[i];
    LatticeTrajectory traj;
    traj.lateral_offset = r_target;

    // 生成轨迹点：纵向 s 采样，横向偏移从 0 平滑过渡到 r_target
    for (int k = 0; k <= p_.samples; ++k) {
      double s = p_.lookahead * static_cast<double>(k) / p_.samples;
      // 余弦过渡：s=0 时 r=0，s=lookahead 时 r=r_target，端点导数为 0
      double t = static_cast<double>(k) / p_.samples;
      double r = 0.5 * r_target * (1.0 - std::cos(M_PI * t));

      double theta = s0_angle + s / p_.ring_radius;
      double nx = std::cos(theta);  // 径向向外
      double ny = std::sin(theta);

      Vec2 center{ p_.ring_radius * std::cos(theta),
                   p_.ring_radius * std::sin(theta) };
      traj.path.push_back(Vec2{ center.x + r * nx, center.y + r * ny });
    }

    // 代价 = 障碍物代价 + 横向偏移代价
    traj.cost = obstacle_cost(traj, obstacles)
              + p_.lateral_cost_gain * std::fabs(r_target);

    // 建议速度：靠近障碍物减速，否则按最大速度
    double min_d = std::numeric_limits<double>::infinity();
    for (const auto& ob : obstacles) {
      for (const auto& pt : traj.path) {
        double dx = pt.x - ob.position.x;
        double dy = pt.y - ob.position.y;
        double d = std::sqrt(dx * dx + dy * dy) - ob.radius;
        if (d < min_d) min_d = d;
      }
    }
    traj.clearance = (min_d == std::numeric_limits<double>::infinity())
                         ? p_.obstacle_safe_dist : min_d;
    // 沿轨迹的最小间距小于小车半宽则视为不可通行（相撞）。
    // 注意：这里的 clearance 是「沿所选路径」的最小距离，而非车正前方直线距离，
    // 因此窄门侧墙/绕桩桩桶这类可绕行/可穿过的障碍不会被误判为堵死。
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

  // 标记最优轨迹
  if (best_idx >= 0 && best_idx < static_cast<int>(out.size())) {
    out[static_cast<size_t>(best_idx)].selected = true;
  }
}

}  // namespace sdc
