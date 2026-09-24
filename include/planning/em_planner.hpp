#ifndef SELF_DRIVING_CAR_PLANNING_EM_PLANNER_HPP
#define SELF_DRIVING_CAR_PLANNING_EM_PLANNER_HPP

#include <vector>

#include "planning/lattice_planner.hpp"  // 复用 Vec2 / Obstacle / LatticeTrajectory

namespace sdc {

/// EM 规划器参数。
struct EmPlannerParams {
  double lookahead = 25.0;           // 规划视距（纵向弧长，米）
  int s_samples = 30;                // 纵向采样点数
  double max_lateral = 2.4;          // 最大横向偏移（米）
  double lateral_step = 0.6;         // 横向候选间隔（米）
  double obstacle_cost_gain = 40.0;  // 障碍物距离代价增益
  double lateral_cost_gain = 1.2;    // 横向偏移代价增益
  double smoothness_gain = 0.35;     // 轨迹平滑（曲率/横向变化）代价增益
  double obstacle_safe_dist = 2.0;   // 期望与障碍物的安全距离（米）
  double max_speed = 3.0;            // 默认最大速度
  double ring_radius = 26.0;         // 环道半径（用于 Frenet→Cartesian 换算）
};

/// 基于 Frenet 框架 + EM 思想的局部规划器。
///
/// 采用「E 步采样 → M 步选择」的两阶段结构，模拟 EM 规划的思路：
///   - E 步（Expectation / 采样）：在 Frenet 坐标系下，沿环道纵向弧长 s
///     均匀采样，并对不同目标横向偏移 d_target 采样多条候选轨迹；每条轨迹
///     用五阶多项式（quintic）在 s 上生成从当前横向偏移/导数到目标偏移
///     的光滑过渡，再映射回 Cartesian 世界坐标。
///   - M 步（Maximization / 评估选择）：综合「障碍物代价 + 横向偏移代价 +
///     平滑（曲率变化）代价」对候选轨迹打分，选择代价最小的轨迹作为
///     当前最优局部路径。
///
/// 输出格式复用 LatticeTrajectory，因此仿真节点无需改动即可复用可视化与
/// 决策逻辑（候选路径灰色、最优路径绿色）。
class EmPlanner {
 public:
  explicit EmPlanner(const EmPlannerParams& p = EmPlannerParams());

  /// 规划局部轨迹。
  /// @param pose_x, pose_y  小车世界坐标
  /// @param yaw             小车朝向（前进方向，弧度）
  /// @param obstacles       活跃障碍物列表（世界坐标）
  /// @param out             输出的候选轨迹（含选中的最优轨迹）
  void plan(double pose_x,
            double pose_y,
            double yaw,
            const std::vector<Obstacle>& obstacles,
            std::vector<LatticeTrajectory>& out) const;

  /// 默认最大车速（供上层决策参考）。
  double max_speed() const {
    return p_.max_speed;
  }

 private:
  /// 环道上某纵向弧长 s 对应的中心线点（世界坐标）。
  Vec2 center_at(double s0_angle, double s) const;
  /// 由 quintic 横向多项式系数在 s∈[0, lookahead] 上生成横向偏移。
  void build_profile(double d0,
                     double d0_prime,
                     double d_target,
                     double lookahead,
                     int samples,
                     std::vector<double>& d_profile) const;
  /// 计算一条候选轨迹的总代价。
  double evaluate(const LatticeTrajectory& traj, const std::vector<Obstacle>& obstacles) const;

  EmPlannerParams p_;
};

}  // namespace sdc

#endif  // SELF_DRIVING_CAR_PLANNING_EM_PLANNER_HPP
