#pragma once
#include "planning/planner.hpp"

namespace sdc::planning {
class ReferencePathPlanner final : public Planner {
 public:
  struct Config {
    double spacing{0.25};
    double horizon{30.0};
    double obstacle_margin{0.8};
    bool smooth_corners{false};
  };
  ReferencePathPlanner();
  explicit ReferencePathPlanner(Config config);
  PlanningResult plan(const PlanningInput& input) override;

 private:
  Config config_;
};
}  // namespace sdc::planning
