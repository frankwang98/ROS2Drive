#pragma once

#include "planning/planner.hpp"

namespace sdc::planning {

// A two-lane circular Frenet planner using quintic lateral d(s) curves.
// It is intentionally separate
// from the scenario-agnostic ReferencePathPlanner: lane semantics belong to
// the ring teaching scenario, not to the general Runtime contract.
class RingLanePlanner final : public Planner {
 public:
  struct Config {
    double lane_width{3.0};
    double spacing{0.25};
    // Keep the local horizon inside the distance where an 8 m lane change
    // can be completed; obstacles beyond it are handled on the next cycle.
    double horizon{24.0};
    double change_length{8.0};
    // Keep adjacent 3 m lanes independent: obstacle(0.8) + vehicle(0.6)
    // + margin(0.3) remains below the lane-centre separation.
    double obstacle_margin{0.3};
    double vehicle_radius{0.6};
    // -1 selects the obstacle-driven lane policy; [0, lane_width] requests
    // an outward-positive Frenet d. Both modes retain collision checking.
    double target_d{-1.0};
    double maximum_curvature{0.30};
  };

  RingLanePlanner();
  explicit RingLanePlanner(Config config);
  PlanningResult plan(const PlanningInput& input) override;
  // Invalid requests leave the current target unchanged.
  bool setTargetD(double target_d);

 private:
  enum class Maneuver { kKeepLeft, kChangeRight, kKeepRight, kChangeLeft };
  Config config_;
  Maneuver maneuver_{Maneuver::kKeepLeft};
  bool passed_left_obstacle_{false};
};

}  // namespace sdc::planning
