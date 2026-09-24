#ifndef SELF_DRIVING_CAR_SIM_OBSTACLE_MANAGER_HPP
#define SELF_DRIVING_CAR_SIM_OBSTACLE_MANAGER_HPP

#include <random>
#include <vector>

#include "sim/scene_types.hpp"

namespace sdc {

/// 环道上的障碍物实体（沿环道分布，世界坐标由角度换算得到）。
struct RingObstacle {
  double angle = 0.0;      // 在环道上的角度位置（弧度）
  double lateral = 0.0;    // 横向偏移（相对中心线，米，负=向内/正=向外）
  double radius = 1.0;     // 障碍物半径（米）
  double remaining = 0.0;  // 剩余存活时间（秒）
  double x = 0.0;          // 世界坐标
  double y = 0.0;
};

/// 随机障碍物管理器。
///
/// 在环形道路上按一定概率随机生成障碍物（位置、大小、存活时长随机），
/// 存活期结束后自动移除。可把当前活跃障碍物转换为世界坐标
/// Obstacle 列表，也便于在 RViz 中绘制。
class ObstacleManager {
 public:
  explicit ObstacleManager(double ring_radius = 25.0,
                           double road_width = 6.0,
                           unsigned seed = 20240812);

  /// 每仿真步更新一次：按概率生成新障碍物，并推进存活时间。
  void update(double dt);

  /// 当前活跃障碍物数量。
  size_t size() const {
    return obstacles_.size();
  }

  /// 活跃障碍物（world 坐标）。
  const std::vector<RingObstacle>& obstacles() const {
    return obstacles_;
  }

  /// 转为世界坐标障碍物列表。
  std::vector<Obstacle> to_planner_obstacles() const;

 private:
  /// 生成一个新障碍物（位置在环道前方某个随机角度）。
  void spawn();

  double ring_radius_;
  double road_width_;
  double half_lane_;  // 横向偏移允许范围 = 半个车道宽 * 0.8

  std::mt19937 rng_;
  std::uniform_real_distribution<double> angle_dist_;
  std::uniform_real_distribution<double> lateral_dist_;
  std::uniform_real_distribution<double> radius_dist_;
  std::uniform_real_distribution<double> lifetime_dist_;
  std::uniform_real_distribution<double> spawn_prob_;

  std::vector<RingObstacle> obstacles_;
};

}  // namespace sdc

#endif  // SELF_DRIVING_CAR_SIM_OBSTACLE_MANAGER_HPP
