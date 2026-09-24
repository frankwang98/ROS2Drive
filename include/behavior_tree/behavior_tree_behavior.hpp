#pragma once

#include <string>

#include "behavior/behavior_manager.hpp"
#include "behavior_tree/behavior_tree_planner.hpp"

namespace sdc {
class BehaviorTreeBehavior final : public behavior::BehaviorManager {
 public:
  BehaviorTreeBehavior(std::string xml_path, std::string tree_id);
  behavior::BehaviorDecision decide(const behavior::BehaviorInput& input) override;
  void reset() override {}
  bool initialized() const {
    return planner_.initialized();
  }
  bool configurationAccepted() const {
    return configuration_accepted_;
  }
  const std::string& lastError() const {
    return planner_.last_error();
  }

 private:
  BehaviorTreePlanner planner_;
  bool configuration_accepted_{false};
};
}  // namespace sdc
