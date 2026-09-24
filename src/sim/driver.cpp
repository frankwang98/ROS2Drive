#include "sim/driver.hpp"

#include <algorithm>
#include <cmath>

namespace sdc {

double AutoDriver::target_speed_for_action(Action a) {
  switch (a) {
    case Action::kAccelerate:
      return 3.0;
    case Action::kCruise:
      return 2.0;
    case Action::kBrake:
      return 0.5;
    case Action::kStop:
      return 0.0;
    default:
      return 0.0;
  }
}

const char* AutoDriver::planning_algorithm_name(PlanningAlgorithm a) {
  switch (a) {
    case PlanningAlgorithm::kLattice:
      return "Lattice";
    case PlanningAlgorithm::kEm:
      return "EM";
    default:
      return "?";
  }
}

const char* AutoDriver::lateral_algorithm_name(LateralAlgorithm a) {
  switch (a) {
    case LateralAlgorithm::kStanley:
      return "Stanley";
    case LateralAlgorithm::kLqr:
      return "LQR";
    case LateralAlgorithm::kMpc:
      return "MPC";
    default:
      return "?";
  }
}

AutoDriver::AutoDriver() = default;

void AutoDriver::set_use_behavior_tree(bool on) {
  if (on && !behavior_tree_.initialized()) {
    behavior_tree_.init();
  }
  use_behavior_tree_ = on;
}

void AutoDriver::reset(const ScenarioMap* map) {
  map_ = map;
  speed_ = 0.0;
  reversing_ = false;
  candidates_.clear();
  steering_controller_.reset();
  lqr_controller_.reset();
  mpc_controller_.reset();
  velocity_controller_.reset();
  if (map)
    map->reset(car_);
}

void AutoDriver::run_planner(double car_x,
                             double car_y,
                             double car_yaw,
                             const std::vector<Obstacle>& obstacles) {
  if (planning_algo_ == PlanningAlgorithm::kEm) {
    em_planner_.plan(car_x, car_y, car_yaw, obstacles, candidates_);
  } else {
    lattice_planner_.plan(car_x, car_y, car_yaw, obstacles, candidates_);
  }
}

double AutoDriver::compute_steer(
    double car_x, double car_y, double car_yaw, const Vec2& target, double speed_cmd, double dt) {
  switch (lateral_algo_) {
    case LateralAlgorithm::kLqr:
      return lqr_controller_.compute(car_x, car_y, car_yaw, target.x, target.y, speed_cmd);
    case LateralAlgorithm::kMpc:
      return mpc_controller_.compute(car_x, car_y, car_yaw, target.x, target.y, speed_cmd, dt);
    case LateralAlgorithm::kStanley:
    default:
      return steering_controller_.compute(car_x, car_y, car_yaw, target.x, target.y, speed_cmd);
  }
}

double AutoDriver::front_obstacle_distance(const std::vector<Obstacle>& obs,
                                           double car_x,
                                           double car_y,
                                           double car_yaw) const {
  constexpr double LIDAR_RANGE = 35.0;
  double fx = std::cos(car_yaw);
  double fy = std::sin(car_yaw);
  double min_d = LIDAR_RANGE;
  for (const auto& ob : obs) {
    double dx = ob.position.x - car_x;
    double dy = ob.position.y - car_y;
    double dist = std::hypot(dx, dy) - ob.radius;
    if (dist < 0)
      dist = 0.0;
    if (dist < min_d && (dx * fx + dy * fy) > 0.2) {
      min_d = dist;
    }
  }
  return min_d;
}

AutoDriver::StepResult AutoDriver::step(const Vec2& target, bool allow_reverse, double dt) {
  StepResult res;

  double car_x = car_.x(), car_y = car_.y(), car_yaw = car_.yaw();

  // 1. 障碍物来源：地图静态边界 + 外部注入的动态障碍（环道随机障碍）
  auto obstacles = map_->to_obstacles();
  obstacles.insert(obstacles.end(), extra_obstacles_.begin(), extra_obstacles_.end());

  // 2. 局部规划避障：按当前规划算法生成候选轨迹并选出最优（selected）轨迹。
  run_planner(car_x, car_y, car_yaw, obstacles);

  // 3. 前方距离感知（仅用于上报/可视化，不再直接决定是否停车）。
  double front_dist = front_obstacle_distance(obstacles, car_x, car_y, car_yaw);

  // 4. 决策：以 Lattice 选中的局部路径是否可通行为准，而不是以车正前方
  //    直线距离为准。窄门侧墙 / 绕桩桩桶这类「可绕行/可穿过」的障碍，
  //    在选中路径上并不会挡住去路，因此不会误判为必须停车；只有当
  //    选中路径也穿障（blocked）时才停车。
  double plan_speed = lattice_planner_.max_speed();
  bool path_blocked = false;
  for (const auto& c : candidates_) {
    if (c.selected) {
      plan_speed = c.speed;
      path_blocked = c.blocked;
      break;
    }
  }

  Action action;
  if (path_blocked) {
    action = Action::kStop;  // 无可通行路径，停车
  } else if (use_behavior_tree_) {
    // 行为树（BehaviorTree.CPP v3）基础行为切换：
    // 以选中路径的建议速度换算成前方等效距离，驱动行为树决策。
    // plan_speed 低说明靠近障碍，将其映射为距离供条件节点判定。
    double bt_dist = 10.0;
    if (plan_speed < 1.0)
      bt_dist = 1.0;  // 接近停车
    else if (plan_speed < 2.0)
      bt_dist = 3.0;  // 减速区间
    else if (plan_speed < 2.5)
      bt_dist = 6.0;  // 巡航区间
    action = behavior_tree_.tick(bt_dist);
  } else {
    action = decision_maker_.decide_by_speed(plan_speed);
  }

  // 5. 速度控制
  //    抵达目标附近则减速停车；否则按决策速度。
  double target_speed = target_speed_for_action(action);
  // 若 Lattice 因靠近障碍已建议减速，则沿用更保守的速度。
  if (!path_blocked)
    target_speed = std::min(target_speed, plan_speed);
  double d_to_goal = std::hypot(target.x - car_x, target.y - car_y);
  if (d_to_goal < 2.5)
    target_speed = std::min(target_speed, 0.8);
  if (d_to_goal < 0.8)
    target_speed = 0.0;

  double speed_cmd = velocity_controller_.update(target_speed, speed_, dt);

  // 6. 转向：按当前横向控制算法跟踪目标点
  double steer = compute_steer(car_x, car_y, car_yaw, target, speed_cmd, dt);

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

double AutoDriver::max_steer_rad() const {
  // 手动驾驶转向范围：与阿克曼模型最大转角一致（约 ±0.55 rad）
  return 0.55;
}

double AutoDriver::manual_step(double throttle, double steer_cmd, double dt) {
  // 手动模式：直接由键盘指令控制车速与转向，绕过自动决策/避障规划。
  // throttle: 目标油门 (-1..1)。>0 前进，<0 倒车。
  //   前进：车速向 max_speed*throttle 逼近；倒车：向反方向逼近。
  constexpr double kMaxSpeed = 4.0;  // 手动最高车速（m/s）
  constexpr double kAccel = 2.5;     // 加速/制动响应（m/s²）

  // 1. 速度指令
  double target_speed = throttle * kMaxSpeed;
  double delta = target_speed - speed_;
  double max_dv = kAccel * dt;
  if (std::fabs(delta) > max_dv) {
    speed_ += (delta > 0.0 ? max_dv : -max_dv);
  } else {
    speed_ = target_speed;
  }

  // 2. 转向指令（-1..1 -> 前轮转角，平滑逼近避免突变）
  double steer = steer_cmd * max_steer_rad();
  car_.update(speed_, steer, dt);

  // 3. 记录倒车状态（用于可视化）
  reversing_ = (speed_ < -0.1);

  return speed_;
}

void AutoDriver::apply_control(const domain::ControlCommand& command, double dt) {
  const double requested_speed =
      (command.emergency_stop || command.brake >= 0.99) ? 0.0 : command.target_speed;
  speed_ = velocity_controller_.update(requested_speed, speed_, dt);
  car_.update(speed_, command.steering_angle, dt);
  speed_ = car_.speed();
  reversing_ = speed_ < -0.1;
}

}  // namespace sdc
