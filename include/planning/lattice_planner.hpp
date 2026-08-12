#ifndef SELF_DRIVING_CAR_PLANNING_LATTICE_PLANNER_HPP
#define SELF_DRIVING_CAR_PLANNING_LATTICE_PLANNER_HPP

#include <vector>

namespace sdc {

/// 二维点。
struct Vec2 {
  double x = 0.0;
  double y = 0.0;
};

/// 障碍物（世界坐标，圆形近似）。
struct Obstacle {
  Vec2 position;
  double radius = 1.0;  // 障碍物半径（米）
};

/// 一条局部候选轨迹（世界坐标点序列）。
struct LatticeTrajectory {
  std::vector<Vec2> path;      // 世界坐标下的轨迹点
  double lateral_offset = 0.0; // 目标横向偏移（米）
  double cost = 0.0;           // 该轨迹的代价（越小越优）
  double speed = 0.0;          // 建议沿该轨迹行驶的速度（m/s）
  bool selected = false;       // 是否被选为最优轨迹
};

/// 局部规划器参数。
struct LatticeParams {
  double lookahead = 25.0;    // 规划视距（纵向距离，米）
  int    samples = 20;        // 纵向采样点数
  double max_lateral = 2.2;   // 最大横向偏移（米）
  double lateral_step = 0.55; // 横向候选间隔（米）
  double obstacle_cost_gain = 40.0;  // 障碍物距离代价增益
  double lateral_cost_gain = 1.5;    // 横向偏移代价增益
  double obstacle_safe_dist = 2.0;   // 期望与障碍物的安全距离（米）
  double max_speed = 3.0;     // 默认最大速度
  double ring_radius = 25.0;  // 环道半径（用于坐标换算）
};

/// 基于 lattice 采样的局部规划避障器。
///
/// 在环道（圆弧）上以当前小车位置为起点，沿前进方向对纵向距离 s 采样，
/// 并对不同目标横向偏移 r 采样多条候选轨迹；每条轨迹由当前横向偏移
/// 平滑过渡到目标横向偏移。根据与障碍物的距离、横向偏移大小评估代价，
/// 选择代价最小的轨迹作为避障局部路径，并给出建议行驶速度。
class LatticePlanner {
 public:
  explicit LatticePlanner(const LatticeParams& p = LatticeParams());

  /// 规划局部轨迹。
  /// @param pose_x, pose_y  小车世界坐标
  /// @param yaw             小车朝向（前进方向，弧度）
  /// @param obstacles       活跃障碍物列表（世界坐标）
  /// @param out             输出的候选轨迹（含选中的最优轨迹）
  void plan(double pose_x, double pose_y, double yaw,
            const std::vector<Obstacle>& obstacles,
            std::vector<LatticeTrajectory>& out) const;

 private:
  /// 环道上某纵向弧长 s 对应的小车中心点世界坐标。
  Vec2 center_at(double s0_angle, double s) const;
  /// 计算轨迹到障碍物的最小距离代价。
  double obstacle_cost(const LatticeTrajectory& traj,
                       const std::vector<Obstacle>& obstacles) const;

  LatticeParams p_;
};

}  // namespace sdc

#endif  // SELF_DRIVING_CAR_PLANNING_LATTICE_PLANNER_HPP
