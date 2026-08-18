#ifndef SELF_DRIVING_CAR_SIM_DRIVER_HPP
#define SELF_DRIVING_CAR_SIM_DRIVER_HPP

#include <vector>

#include "behavior_tree/behavior_tree_planner.hpp"
#include "control/lqr_controller.hpp"
#include "control/mpc_controller.hpp"
#include "control/steering_controller.hpp"
#include "control/velocity_controller.hpp"
#include "decision/decision_maker.hpp"
#include "model/ackermann_model.hpp"
#include "planning/em_planner.hpp"
#include "planning/lattice_planner.hpp"
#include "sim/map.hpp"

namespace sdc {

/// 自动驾驶执行器。
///
/// 把原仿真节点主循环里的「感知 → 局部规划避障 → 决策 → 速度控制
/// → 转向控制 → 阿克曼积分」封装成通用接口，使小车能**自动循迹行驶**。
///
/// 规控算法均可在运行时切换：
///   - 局部规划：Lattice Planner（默认）/ EM Planner
///   - 横向控制：Stanley（默认）/ LQR / MPC
///   - 速度控制：PID / Bang-Bang / Ramp
///
/// - 环道模式：target 为沿切线前方的虚拟点（无限行驶，避障 + 沿环）。
///
/// 倒车：speed_cmd 可传负值（阿克曼模型支持），用于倒车行驶。
class AutoDriver {
 public:
  // ========== 规控算法类型 ==========
  enum class PlanningAlgorithm { kLattice = 0, kEm = 1 };
  enum class LateralAlgorithm { kStanley = 0, kLqr = 1, kMpc = 2 };

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
  /// @param target 期望行驶目标点（世界坐标）
  /// @param allow_reverse 是否允许倒车（当目标在车体后方时）
  /// @param dt 时间步长（秒）
  StepResult step(const Vec2& target, bool allow_reverse, double dt);

  /// 手动驾驶：直接由键盘指令控制（W/S 油门/倒车，A/D 转向）。
  /// @param throttle 油门（>0 前进加速，<0 倒车），-1..1
  /// @param steer_cmd 转向指令（-1..1，负=左 正=右）
  /// @param dt 时间步长
  /// @return 当前车速（m/s）
  double manual_step(double throttle, double steer_cmd, double dt);

  // ========== 规控算法切换 ==========
  static const char* planning_algorithm_name(PlanningAlgorithm a);
  static const char* lateral_algorithm_name(LateralAlgorithm a);

  void set_planning_algorithm(PlanningAlgorithm a) { planning_algo_ = a; }
  PlanningAlgorithm planning_algorithm() const { return planning_algo_; }

  void set_lateral_algorithm(LateralAlgorithm a) { lateral_algo_ = a; }
  LateralAlgorithm lateral_algorithm() const { return lateral_algo_; }

  // ========== 行为树（BehaviorTree.CPP v3）基础行为切换 ==========
  /// 是否启用行为树进行基础行为决策（加速/巡航/减速/停车）。
  /// 启用前会自动初始化行为树；未安装行为树库时退化为规则决策。
  void set_use_behavior_tree(bool on);
  bool use_behavior_tree() const { return use_behavior_tree_; }
  /// 最近一次行为树给出的行为（供可视化 / 日志）。
  Action behavior_tree_action() const { return behavior_tree_.last_action(); }

  // 直接访问小车状态（用于可视化 / TF）
  CarState& car() { return car_; }
  const CarState& car() const { return car_; }

  double speed() const { return speed_; }

  /// 手动模式下是否处于倒车（用于可视化箭头）。
  bool reversing() const { return reversing_; }

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
  /// 按当前规划算法执行局部规划，填充 candidates_。
  void run_planner(double car_x, double car_y, double car_yaw,
                   const std::vector<Obstacle>& obstacles);
  /// 按当前横向控制算法计算前轮转角。
  double compute_steer(double car_x, double car_y, double car_yaw,
                       const Vec2& target, double speed_cmd, double dt);

  const ScenarioMap* map_{nullptr};
  CarState car_;
  DecisionMaker decision_maker_;
  VelocityController velocity_controller_;
  SteeringController steering_controller_;
  LqrController lqr_controller_;
  MpcController mpc_controller_;
  LatticePlanner lattice_planner_;
  EmPlanner em_planner_;
  BehaviorTreePlanner behavior_tree_;

  PlanningAlgorithm planning_algo_{PlanningAlgorithm::kLattice};
  LateralAlgorithm lateral_algo_{LateralAlgorithm::kStanley};
  bool use_behavior_tree_{false};

  double speed_{0.0};
  bool   reversing_{false};
  std::vector<LatticeTrajectory> candidates_;
  std::vector<Obstacle> extra_obstacles_;

  // 避障：返回最近障碍距离（仅考虑车前方）
  double front_obstacle_distance(const std::vector<Obstacle>& obs,
                                 double car_x, double car_y, double car_yaw) const;

  // 沿环/路径取前视目标点
  static double target_speed_for_action(Action a);
  // 手动模式下用最大转向角
  double max_steer_rad() const;
};

}  // namespace sdc

#endif  // SELF_DRIVING_CAR_SIM_DRIVER_HPP
