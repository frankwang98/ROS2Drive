#ifndef SELF_DRIVING_CAR_SIM_DRIVER_HPP
#define SELF_DRIVING_CAR_SIM_DRIVER_HPP

#include <vector>

#include "control/steering_controller.hpp"
#include "control/velocity_controller.hpp"
#include "decision/decision_maker.hpp"
#include "model/ackermann_model.hpp"
#include "planning/lattice_planner.hpp"
#include "sim/map.hpp"

namespace sdc {

/// 自动驾驶执行器。
///
/// 把原仿真节点主循环里的「感知 → Lattice 局部规划避障 → 决策 → 速度控制
/// → Stanley 转向 → 阿克曼积分」封装成通用接口，使小车能**自动循迹行驶**。
///
/// - 环道模式：target 为沿切线前方的虚拟点（无限行驶，避障 + 沿环）。
/// - 考试模式：target 为当前科目目标点（如库位中心），抵达即判完成。
///
/// 倒车：speed_cmd 可传负值（阿克曼模型支持），用于倒车入库等场景。
class AutoDriver {
 public:
  struct StepResult {
    double speed = 0.0;       // 当前车速（m/s）
    double front_dist = 0.0;  // 前方最近障碍物距离（m）
    Action action = Action::kCruise;
    bool goal_reached = false;
  };

  AutoDriver();

  /// 设置当前地图（决定场景布局与障碍物来源）。
  void set_map(const ScenarioMap* map) { map_ = map; }

  /// 额外的动态障碍物（如环道随机障碍），与地图静态边界一并参与避障。
  void set_extra_obstacles(const std::vector<Obstacle>& obs) { extra_obstacles_ = obs; }

  /// 推进一个仿真步。
  /// @param target 期望行驶目标点（世界坐标）；考试中为科目目标
  /// @param allow_reverse 是否允许倒车（倒车入库设为 true）
  /// @param dt 时间步长（秒）
  StepResult step(const Vec2& target, bool allow_reverse, double dt);

  // 直接访问小车状态（用于可视化 / TF）
  CarState& car() { return car_; }
  const CarState& car() const { return car_; }

  double speed() const { return speed_; }

  /// 重置小车到地图起点并清零状态。
  void reset(const ScenarioMap* map);

  /// 暴露最近一次 Lattice 候选（供可视化）。
  const std::vector<LatticeTrajectory>& last_candidates() const {
    return candidates_;
  }

  const DecisionMaker& decision_maker() const { return decision_maker_; }

  /// 访问速度控制器（外部可切换控制算法）。
  VelocityController& velocity_ctrl() { return velocity_controller_; }
  const VelocityController& velocity_ctrl() const { return velocity_controller_; }

 private:
  const ScenarioMap* map_{nullptr};
  CarState car_;
  DecisionMaker decision_maker_;
  VelocityController velocity_controller_;
  SteeringController steering_controller_;
  LatticePlanner lattice_planner_;

  double speed_{0.0};
  std::vector<LatticeTrajectory> candidates_;
  std::vector<Obstacle> extra_obstacles_;

  // 避障：返回最近障碍距离（仅考虑车前方）
  double front_obstacle_distance(const std::vector<Obstacle>& obs,
                                 double car_x, double car_y, double car_yaw) const;

  // 沿环/路径取前视目标点（考试时不使用，考试直接用科目目标）
  static double target_speed_for_action(Action a);
};

}  // namespace sdc

#endif  // SELF_DRIVING_CAR_SIM_DRIVER_HPP
