#pragma once

#include <string>
#include <vector>

#include "domain/autonomy_types.hpp"

namespace sdc::behavior {
struct BehaviorInput {
  domain::VehicleState vehicle;
  domain::Mission mission;
  std::vector<domain::Obstacle> obstacles;
};
struct BehaviorDecision {
  double speed_limit{0.0};
  bool stop_required{false};
  std::string behavior{"CRUISE"};
  std::string reason;
};
class BehaviorManager {
 public:
  virtual ~BehaviorManager() = default;
  virtual BehaviorDecision decide(const BehaviorInput& input) = 0;
  virtual void reset() = 0;
};
class PassthroughBehaviorManager final : public BehaviorManager {
 public:
  BehaviorDecision decide(const BehaviorInput& input) override {
    return {input.mission.speed_limit, false, "CRUISE", ""};
  }
  void reset() override {}
};
}  // namespace sdc::behavior
