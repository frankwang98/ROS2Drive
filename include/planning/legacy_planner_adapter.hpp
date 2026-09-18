#pragma once

#include "planning/planner.hpp"
#include "planning/lattice_planner.hpp"
#include "planning/em_planner.hpp"

namespace sdc::planning {

class LegacyPlannerAdapter final : public Planner {
 public:
  enum class Type { kLattice, kEm };
  explicit LegacyPlannerAdapter(Type type);
  PlanningResult plan(const PlanningInput& input) override;

 private:
  Type type_;
  LatticePlanner lattice_;
  EmPlanner em_;
};

}  // namespace sdc::planning
