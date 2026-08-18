#include "behavior_tree/behavior_tree_planner.hpp"

#include <string>
#include <vector>

#include "decision/decision_maker.hpp"

// ============================================================
// BehaviorTree.CPP v3 依赖。
//
// 说明：sdc_core 保持「纯逻辑、弱依赖」特性。若构建环境未安装
//   behaviors_ccp_v3，则本模块退化为传统规则决策（DecisionMaker），
//   行为树库为可选能力。安装后即可启用完整的行为树实现。
//   （通过 CMake 传入 -DBEHAVIORTREE_CPP_V3_FOUND=1 启用）
// ============================================================
#ifdef BEHAVIORTREE_CPP_V3_FOUND
#include <behaviortree_cpp_v3/action_node.h>
#include <behaviortree_cpp_v3/behavior_tree.h>
#include <behaviortree_cpp_v3/blackboard.h>
#include <behaviortree_cpp_v3/bt_factory.h>
#include <behaviortree_cpp_v3/condition_node.h>
#include <behaviortree_cpp_v3/tree_node.h>
#endif

namespace sdc {

// ============================================================
// 行为树 XML 描述
// ============================================================
// 结构：Fallback（任一子节点成功即成功，从上到下依次尝试）
//   - EmergencyStop 条件成立 -> 停车（成功）
//   - SlowDown 条件成立      -> 减速（成功）
//   - Cruise 条件成立        -> 匀速巡航（成功）
//   - Accelerate 兜底        -> 加速（成功）
//
// 条件节点与行为节点通过黑板（blackboard）共享 `front_dist` 与 `action`。
// ============================================================
const char* BehaviorTreePlanner::xml() {
  return R"(
<root BTCPP_format="4">
  <BehaviorTree ID="BasicBehaviorSwitch">
    <Fallback>
      <EmergencyStop front_dist="{front_dist}"/>
      <SlowDown     front_dist="{front_dist}"/>
      <Cruise       front_dist="{front_dist}"/>
      <Accelerate/>
    </Fallback>
  </BehaviorTree>
</root>
)";
}

namespace {

#ifdef BEHAVIORTREE_CPP_V3_FOUND

// ---------- 条件节点：按距离判定，命中后设置对应行为 ----------
class DistanceCondition : public BT::ConditionNode {
 public:
  DistanceCondition(const std::string& name,
                    const BT::NodeConfiguration& config)
      : BT::ConditionNode(name, config) {}

  static BT::PortsList providedPorts() {
    return {BT::InputPort<double>("front_dist"),
            BT::OutputPort<int>("action")};
  }

 protected:
  // 子类实现具体阈值
  virtual double threshold() const = 0;
  virtual int action_value() const = 0;

  BT::NodeStatus tick() override {
    double front_dist = 0.0;
    if (!getInput("front_dist", front_dist)) {
      return BT::NodeStatus::FAILURE;
    }
    if (front_dist < threshold()) {
      setOutput("action", action_value());
      return BT::NodeStatus::SUCCESS;
    }
    return BT::NodeStatus::FAILURE;
  }
};

class EmergencyStop : public DistanceCondition {
 public:
  using DistanceCondition::DistanceCondition;
  double threshold() const override { return 1.5; }
  int action_value() const override { return static_cast<int>(Action::kStop); }
};

class SlowDown : public DistanceCondition {
 public:
  using DistanceCondition::DistanceCondition;
  double threshold() const override { return 4.0; }
  int action_value() const override { return static_cast<int>(Action::kBrake); }
};

class Cruise : public DistanceCondition {
 public:
  using DistanceCondition::DistanceCondition;
  double threshold() const override { return 8.0; }
  int action_value() const override { return static_cast<int>(Action::kCruise); }
};

// ---------- 动作节点：无条件加速（兜底） ----------
class AccelerateAction : public BT::SyncActionNode {
 public:
  AccelerateAction(const std::string& name,
                   const BT::NodeConfiguration& config)
      : BT::SyncActionNode(name, config) {}

  static BT::PortsList providedPorts() {
    return {BT::OutputPort<int>("action")};
  }

  BT::NodeStatus tick() override {
    setOutput("action", static_cast<int>(Action::kAccelerate));
    return BT::NodeStatus::SUCCESS;
  }
};

#endif  // BEHAVIORTREE_CPP_V3_FOUND

}  // namespace

// ============================================================
// 实现
// ============================================================
BehaviorTreePlanner::BehaviorTreePlanner()
    : factory_(nullptr), tree_(nullptr), initialized_(false),
      last_action_(Action::kCruise) {}

BehaviorTreePlanner::~BehaviorTreePlanner() {
#ifdef BEHAVIORTREE_CPP_V3_FOUND
  // 先销毁 tree，再销毁 factory（tree 内部持有 factory 引用）。
  delete static_cast<BT::Tree*>(tree_);
  tree_ = nullptr;
  delete static_cast<BT::BehaviorTreeFactory*>(factory_);
  factory_ = nullptr;
#endif
}

bool BehaviorTreePlanner::init() {
#ifdef BEHAVIORTREE_CPP_V3_FOUND
  if (initialized_) return true;

  auto* factory = new BT::BehaviorTreeFactory;

  factory->registerNodeType<EmergencyStop>("EmergencyStop");
  factory->registerNodeType<SlowDown>("SlowDown");
  factory->registerNodeType<Cruise>("Cruise");
  factory->registerNodeType<AccelerateAction>("Accelerate");

  // 黑板：默认 front_dist 较大（视为畅通），action 初始为加速。
  auto blackboard = BT::Blackboard::create();
  blackboard->set<double>("front_dist", 10.0);
  blackboard->set<int>("action", static_cast<int>(Action::kAccelerate));

  BT::Tree tree = factory->createTreeFromText(xml(), blackboard);
  // BT::Tree 持有 factory 引用，因此 factory 必须存活到 tree 之后。
  factory_ = factory;
  tree_ = new BT::Tree(std::move(tree));
  initialized_ = true;
  return true;
#else
  // 无行为树库：退化为规则决策，仍保证 init 返回 true（功能可用）。
  initialized_ = false;
  return true;
#endif
}

Action BehaviorTreePlanner::tick(double front_distance) {
#ifdef BEHAVIORTREE_CPP_V3_FOUND
  if (!initialized_ || !tree_) {
    // 未初始化：退化为规则决策。
    last_action_ = DecisionMaker().decide(front_distance);
    return last_action_;
  }

  BT::Tree* tree = static_cast<BT::Tree*>(tree_);
  tree->rootBlackboard()->set<double>("front_dist", front_distance);

  BT::NodeStatus status = tree->tickRoot();
  if (status == BT::NodeStatus::RUNNING || status == BT::NodeStatus::SUCCESS) {
    int action = static_cast<int>(Action::kCruise);
    tree->rootBlackboard()->get("action", action);
    last_action_ = static_cast<Action>(action);
  } else {
    // 行为树整体失败（异常）：采用保守的停车行为。
    last_action_ = Action::kStop;
  }
  return last_action_;
#else
  (void)front_distance;
  // 无行为树库：退化为规则决策。
  last_action_ = DecisionMaker().decide(front_distance);
  return last_action_;
#endif
}

}  // namespace sdc
