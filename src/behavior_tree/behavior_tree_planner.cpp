#include "behavior_tree/behavior_tree_planner.hpp"

#include <string>
#include <memory>
#include <exception>
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
// 结构：所有场景共用 DistancePolicy 子树，仅以 stop_t/slow_t/cruise_t
// 三个黑板参数区分阈值；行为决策完全由 XML 决定，节点层不做硬编码兜底。
//
// DistancePolicy（参数化策略子树）
//   └─ Fallback（任一子节点成功即返回，从上到下依次尝试）
//     ├─ EmergencyStop(front_dist, threshold=stop_t)   紧急停车
//     ├─ SlowDown     (front_dist, threshold=slow_t)   减速
//     ├─ Cruise       (front_dist, threshold=cruise_t) 匀速巡航
//     └─ Accelerate                                  加速（兜底）
//
// 场景 BehaviorTree 仅做一次 SubTree 绑定，把每个场景的阈值写入黑板。
// 真正运行的入口是 RingDemo/PortTransport/MiningHaul/AgricultureRoute 之一。
// ============================================================
const char* BehaviorTreePlanner::xml() {
  return R"(
<root>
  <BehaviorTree ID="DistancePolicy">
    <Fallback>
      <EmergencyStop front_dist="{front_dist}" threshold="{stop_t}" action="{action}"/>
      <SlowDown      front_dist="{front_dist}" threshold="{slow_t}" action="{action}"/>
      <Cruise        front_dist="{front_dist}" threshold="{cruise_t}" action="{action}"/>
      <Accelerate action="{action}"/>
    </Fallback>
  </BehaviorTree>

  <BehaviorTree ID="RingDemo">
    <SubTree ID="DistancePolicy" front_dist="{front_dist}" action="{action}" stop_t="1.5" slow_t="4.0" cruise_t="8.0"/>
  </BehaviorTree>

  <BehaviorTree ID="PortTransport">
    <SubTree ID="DistancePolicy" front_dist="{front_dist}" action="{action}" stop_t="1.2" slow_t="3.5" cruise_t="10.0"/>
  </BehaviorTree>

  <BehaviorTree ID="MiningHaul">
    <SubTree ID="DistancePolicy" front_dist="{front_dist}" action="{action}" stop_t="2.5" slow_t="7.0" cruise_t="14.0"/>
  </BehaviorTree>

  <BehaviorTree ID="AgricultureRoute">
    <SubTree ID="DistancePolicy" front_dist="{front_dist}" action="{action}" stop_t="2.0" slow_t="5.0" cruise_t="12.0"/>
  </BehaviorTree>
</root>
)";
}

namespace {

#ifdef BEHAVIORTREE_CPP_V3_FOUND

// ---------- 条件节点：按距离判定，命中后设置对应行为 ----------
//
// 阈值由 XML/SubTree 参数显式注入（threshold 端口）。节点层不持有任何
// 硬编码默认值：缺少参数会被视为配置错误直接失败，避免静默回退到
// "看起来在跑但其实是错的"状态。
class DistanceCondition : public BT::ConditionNode {
 public:
  DistanceCondition(const std::string& name,
                    const BT::NodeConfiguration& config)
      : BT::ConditionNode(name, config) {}

  static BT::PortsList providedPorts() {
    return {BT::InputPort<double>("front_dist"),
            BT::InputPort<double>("threshold"),
            BT::OutputPort<int>("action")};
  }

 protected:
  // 子类只需声明命中后输出的动作。
  virtual int action_value() const = 0;

  BT::NodeStatus tick() override {
    double front_dist = 0.0;
    if (!getInput("front_dist", front_dist)) {
      return BT::NodeStatus::FAILURE;
    }
    double configured_threshold = 0.0;
    if (!getInput("threshold", configured_threshold)) {
      throw BT::RuntimeError(
          name() + ": missing 'threshold' input port; "
          "threshold must be provided by the BehaviorTree XML / SubTree args.");
    }
    if (front_dist < configured_threshold) {
      setOutput("action", action_value());
      return BT::NodeStatus::SUCCESS;
    }
    return BT::NodeStatus::FAILURE;
  }
};

class EmergencyStop : public DistanceCondition {
 public:
  using DistanceCondition::DistanceCondition;
  int action_value() const override { return static_cast<int>(Action::kStop); }
};

class SlowDown : public DistanceCondition {
 public:
  using DistanceCondition::DistanceCondition;
  int action_value() const override { return static_cast<int>(Action::kBrake); }
};

class Cruise : public DistanceCondition {
 public:
  using DistanceCondition::DistanceCondition;
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

bool BehaviorTreePlanner::init(const std::string& xml_path,
                               const std::string& tree_id) {
#ifdef BEHAVIORTREE_CPP_V3_FOUND
  if (initialized_) return true;

  auto factory = std::make_unique<BT::BehaviorTreeFactory>();

  factory->registerNodeType<EmergencyStop>("EmergencyStop");
  factory->registerNodeType<SlowDown>("SlowDown");
  factory->registerNodeType<Cruise>("Cruise");
  factory->registerNodeType<AccelerateAction>("Accelerate");

  // 黑板：默认 front_dist 较大（视为畅通），action 初始为加速。
  auto blackboard = BT::Blackboard::create();
  blackboard->set<double>("front_dist", 10.0);
  blackboard->set<int>("action", static_cast<int>(Action::kAccelerate));

  try {
    if (tree_id.empty()) throw std::invalid_argument("BehaviorTree ID must not be empty");
    if (xml_path.empty())
      factory->registerBehaviorTreeFromText(xml());
    else
      factory->registerBehaviorTreeFromFile(xml_path);
    BT::Tree tree = factory->createTree(tree_id, blackboard);
    tree_ = new BT::Tree(std::move(tree));
    factory_ = factory.release();
    initialized_ = true;
    last_error_.clear();
    return true;
  } catch (const std::exception& error) {
    last_error_ = error.what();
    initialized_ = false;
    return false;
  }
#else
  (void)xml_path;
  (void)tree_id;
  // 无行为树库：退化为规则决策，仍保证 init 返回 true（功能可用）。
  initialized_ = false;
  last_error_ = "BehaviorTree.CPP v3 unavailable; using rule fallback";
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
