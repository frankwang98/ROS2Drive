#pragma once

#include <string>
#include <vector>

#include "domain/autonomy_types.hpp"

namespace sdc::scenario {

// Scenario-owned limits. Runtime components may translate these into their
// own planner/controller configurations; the scenario does not depend on ROS.
struct VehicleConstraints {
  double maximum_speed{2.0};
  double maximum_acceleration{1.0};
  double maximum_deceleration{1.5};
  double wheelbase{2.0};
};

struct ScenarioDefinition {
  std::string id;
  domain::Pose2D initial_pose;
  std::vector<domain::Pose2D> reference_route;
  std::vector<domain::Obstacle> static_obstacles;
  domain::Mission default_mission;
  VehicleConstraints vehicle_constraints;
  std::string default_behavior_profile;
};

// Validates the portable scenario contract before an adapter installs it.
bool validate(const ScenarioDefinition& definition, std::string& reason);

}  // namespace sdc::scenario
