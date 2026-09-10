#pragma once

#include <string>
#include <vector>
#include "domain/autonomy_types.hpp"

namespace sdc::planning {
struct PlanningInput {
  domain::VehicleState vehicle;
  std::vector<domain::Pose2D> reference_path;
  std::vector<domain::Obstacle> obstacles;
  double speed_limit{2.0};
  double now_s{0.0};
};
struct PlanningResult {
  domain::Trajectory trajectory;
  bool success{false};
  std::string reason;
};
class Planner {
 public:
  virtual ~Planner() = default;
  virtual PlanningResult plan(const PlanningInput& input) = 0;
};
}  // namespace sdc::planning
