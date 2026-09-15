#include "scenario/ring_scenario.hpp"

#include <cmath>
#include <stdexcept>

namespace sdc::scenario {

ScenarioDefinition makeRingScenarioDefinition(double radius, int segments) {
  if (!std::isfinite(radius) || radius <= 0.0 || segments < 8)
    throw std::invalid_argument("ring scenario radius/segments are invalid");

  ScenarioDefinition definition;
  definition.id = "ring_demo";
  definition.initial_pose = {radius, 0.0, M_PI_2};
  definition.reference_route.reserve(static_cast<std::size_t>(segments) + 1);
  for (int i = 0; i <= segments; ++i) {
    const double angle = 2.0 * M_PI * i / segments;
    definition.reference_route.push_back(
        {radius * std::cos(angle), radius * std::sin(angle), angle + M_PI_2});
  }
  definition.default_mission.id = "ring-demo";
  definition.default_mission.type = domain::MissionType::kFollowRoute;
  definition.default_mission.route = definition.reference_route;
  definition.default_mission.speed_limit =
      definition.vehicle_constraints.maximum_speed;
  definition.default_behavior_profile = "RingDemo";
  return definition;
}

}  // namespace sdc::scenario
