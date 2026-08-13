#include "sim/driver.hpp"

#include <cmath>

namespace sdc {

double AutoDriver::target_speed_for_action(Action a) {
  switch (a) {
    case Action::kAccelerate: return 3.0;
    case Action::kCruise:     return 2.0;
    case Action::kBrake:      return 0.5;
    case Action::kStop:       return 0.0;
    default:                  return 0.0;
  }
}

AutoDriver::AutoDriver() = default;

void AutoDriver::reset(const ScenarioMap* map) {
  map_ = map;
  speed_ = 0.0;
  candidates_.clear();
  if (map) map->reset(car_);
}

double AutoDriver::front_obstacle_distance(const std::vector<Obstacle>& obs,
                                           double car_x, double car_y,
                                           double car_yaw) const {
  constexpr double LIDAR_RANGE = 35.0;
  double fx = std::cos(car_yaw);
  double fy = std::sin(car_yaw);
  double min_d = LIDAR_RANGE;
  for (const auto& ob : obs) {
    double dx = ob.position.x - car_x;
    double dy = ob.position.y - car_y;
    double dist = std::hypot(dx, dy) - ob.radius;
    if (dist < 0) dist = 0.0;
    if (dist < min_d && (dx * fx + dy * fy) > 0.2) {
      min_d = dist;
    }
  }
  return min_d;
}

AutoDriver::StepResult AutoDriver::step(const Vec2& target, bool allow_reverse,
                                        double dt) {
  StepResult res;

  double car_x = car_.x(), car_y = car_.y(), car_yaw = car_.yaw();

  // 1. 障碍物来源：地图静态边界 + 外部注入的动态障碍（环道随机障碍）
  auto obstacles = map_->to_obstacles();
  obstacles.insert(obstacles.end(), extra_obstacles_.begin(), extra_obstacles_.end());

  // 2. Lattice 局部规划避障
  lattice_planner_.plan(car_x, car_y, car_yaw, obstacles, candidates_);

  // 3. 前方距离感知
  double front_dist = front_obstacle_distance(obstacles, car_x, car_y, car_yaw);

  // 4. 决策（基于前方距离）
  Action action = decision_maker_.decide(front_dist);

  // 5. 速度控制
  //    考试中：抵达目标附近则减速停车；否则按决策速度。
  double target_speed = target_speed_for_action(action);
  double d_to_goal = std::hypot(target.x - car_x, target.y - car_y);
  if (d_to_goal < 2.5) target_speed = std::min(target_speed, 0.8);
  if (d_to_goal < 0.8) target_speed = 0.0;

  double speed_cmd = velocity_controller_.update(target_speed, speed_, dt);

  // 6. 转向：Stanley 跟踪目标点
  double steer = steering_controller_.compute(
      car_x, car_y, car_yaw, target.x, target.y, speed_cmd);

  // 7. 倒车判定：目标在车体后方（与前进方向夹角 > 90°）且允许倒车
  double to_goal_x = target.x - car_x;
  double to_goal_y = target.y - car_y;
  double dot = to_goal_x * std::cos(car_yaw) + to_goal_y * std::sin(car_yaw);
  if (allow_reverse && dot < 0.0) {
    speed_cmd = -std::fabs(speed_cmd);  // 倒车
  }

  // 8. 阿克曼积分
  car_.update(speed_cmd, steer, dt);
  speed_ = car_.speed();

  // 9. 完成判定
  res.speed = speed_;
  res.front_dist = front_dist;
  res.action = action;
  res.goal_reached = map_->goal_reached(car_);

  return res;
}

}  // namespace sdc
