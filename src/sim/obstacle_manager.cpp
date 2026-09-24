#include "sim/obstacle_manager.hpp"

#include <algorithm>
#include <cmath>

namespace sdc {

ObstacleManager::ObstacleManager(double ring_radius, double road_width, unsigned seed)
    : ring_radius_(ring_radius),
      road_width_(road_width),
      half_lane_(road_width_ / 2.0 * 0.8),
      rng_(seed),
      angle_dist_(0.0, 2.0 * M_PI),
      lateral_dist_(-half_lane_, half_lane_),
      radius_dist_(0.8, 1.4),
      lifetime_dist_(6.0, 16.0),
      spawn_prob_(0.0, 1.0) {}

void ObstacleManager::spawn() {
  RingObstacle ob;
  ob.angle = angle_dist_(rng_);
  ob.lateral = lateral_dist_(rng_);
  ob.radius = radius_dist_(rng_);
  ob.remaining = lifetime_dist_(rng_);

  // 世界坐标：中心线 + 横向偏移（径向向外为正）
  ob.x = (ring_radius_ + ob.lateral) * std::cos(ob.angle);
  ob.y = (ring_radius_ + ob.lateral) * std::sin(ob.angle);

  obstacles_.push_back(ob);
}

void ObstacleManager::update(double dt) {
  // 以一定概率生成新障碍物（避免同一位置堆叠过多，控制总数）
  if (obstacles_.size() < 4 && spawn_prob_(rng_) < 0.02) {
    spawn();
  }

  // 推进存活时间并移除过期障碍物
  for (auto& ob : obstacles_) {
    ob.remaining -= dt;
  }
  obstacles_.erase(std::remove_if(obstacles_.begin(),
                                  obstacles_.end(),
                                  [](const RingObstacle& ob) { return ob.remaining <= 0.0; }),
                   obstacles_.end());
}

std::vector<Obstacle> ObstacleManager::to_planner_obstacles() const {
  std::vector<Obstacle> out;
  out.reserve(obstacles_.size());
  for (const auto& ob : obstacles_) {
    out.push_back(Obstacle{Vec2{ob.x, ob.y}, ob.radius});
  }
  return out;
}

}  // namespace sdc
