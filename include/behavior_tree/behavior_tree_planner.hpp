#ifndef SELF_DRIVING_CAR_BEHAVIOR_TREE_BEHAVIOR_TREE_PLANNER_HPP
#define SELF_DRIVING_CAR_BEHAVIOR_TREE_BEHAVIOR_TREE_PLANNER_HPP

#include <string>

#include "decision/decision_maker.hpp"

// 本模块基于 BehaviorTree.CPP v3 实现「基础行为切换」。
// 通过一棵可读的 XML 行为树，把「前方障碍物距离」映射为小车的基础行为
// （加速 / 匀速巡航 / 减速 / 停车），作为传统规则决策器（DecisionMaker）
// 之外的可选实现，便于学习行为树（BT）的建模方式。

namespace sdc {

/**
 * 基于 BehaviorTree.CPP v3 的基础行为树规划器。
 *
 * 行为树结构（见 behavior_tree_planner.cpp 中的 xml() / scene_driving.xml）：
 *
 *   DistancePolicy（参数化策略子树，被 4 个场景 BT 复用）
 *    └── Fallback（Select）
 *         ├── EmergencyStop（条件：front_dist < stop_t）   -> 停车
 *         ├── SlowDown     （条件：front_dist < slow_t）   -> 减速
 *         ├── Cruise       （条件：front_dist < cruise_t） -> 匀速巡航
 *         └── Accelerate   （无条件，兜底）                -> 加速
 *
 * 每次 tick 行为树时，条件节点读取黑板中的 `front_dist`（前方障碍物距离），
 * 命中后对应的行为节点把 `action`（整型，对应 sdc::Action 枚举值）写回黑板，
 * 最后通过 tick() 返回给调用方。
 *
 * 阈值完全由 XML / SubTree 参数（stop_t / slow_t / cruise_t）显式提供，
 * 节点层不持有任何硬编码默认值；缺参数会立即失败（避免静默回退）。
 *
 * 使用前需要先调用 init() 完成节点注册与行为树加载。
 */
class BehaviorTreePlanner {
 public:
  BehaviorTreePlanner();

  ~BehaviorTreePlanner();

  // 禁止拷贝（持有 BT 对象，内部有非平凡资源）。
  BehaviorTreePlanner(const BehaviorTreePlanner&) = delete;
  BehaviorTreePlanner& operator=(const BehaviorTreePlanner&) = delete;

  /// 初始化行为树。xml_path 为空时使用内置教学默认树；生产场景应传外部 XML。
  /// @return true 表示初始化成功（依赖行为树库可用）。
  bool init(const std::string& xml_path = "", const std::string& tree_id = "RingDemo");

  /// 是否已成功初始化。
  bool initialized() const {
    return initialized_;
  }
  const std::string& last_error() const {
    return last_error_;
  }

  /// 根据前方障碍物距离运行一次行为树，返回当前应执行的行为。
  /// @param front_distance 前方障碍物距离（米）
  /// @return 行为树决定的行为（加速/巡航/减速/停车）
  ///         未初始化时退化为 DecisionMaker 的规则决策。
  Action tick(double front_distance);

  /// 最近一次行为树输出（用于可视化 / 日志）。
  Action last_action() const {
    return last_action_;
  }

  /// 行为树 XML 描述（只读，便于外部查看/调试）。
  static const char* xml();

 private:
  void* factory_;  // BT::BehaviorTreeFactory*（与 tree 生命周期一致）
  void* tree_;     // BT::Tree* 的占位（避免头文件依赖行为树库）
  bool initialized_;
  Action last_action_;
  std::string last_error_;
};

}  // namespace sdc

#endif  // SELF_DRIVING_CAR_BEHAVIOR_TREE_BEHAVIOR_TREE_PLANNER_HPP
