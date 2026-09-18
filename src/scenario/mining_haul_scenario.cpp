#include "scenario/mining_haul_scenario.hpp"

namespace sdc::scenario {

ScenarioDefinition makeMiningHaulScenarioDefinition() {
  ScenarioDefinition definition;
  definition.id = "mining_haul";
  definition.initial_pose = {-32.0, -16.0, 0.0};
  definition.reference_route = {
      {-32.0, -16.0, 0.0, 0.0}, {-12.0, -16.0, 0.0, 0.5},
      {8.0, -10.0, 0.35, 2.0},  {30.0, -4.0, 0.0, 4.0},
      {34.0, 14.0, 1.57, 6.0}};

  // Deterministic roadside hazards; they exercise perception without
  // blocking the reference-route centreline.
  definition.static_obstacles = {
      {"haul-barrier-1", {-4.0, -11.5, 0.0, 1.2}, 1.1, false},
      {"haul-barrier-2", {20.0, -1.0, 0.0, 3.6}, 1.2, false},
      {"dump-zone-cone", {29.0, 11.0, 0.0, 5.8}, 0.8, false}};

  definition.vehicle_constraints.maximum_speed = 1.5;
  definition.vehicle_constraints.maximum_acceleration = 0.6;
  definition.vehicle_constraints.maximum_deceleration = 1.0;
  definition.vehicle_constraints.wheelbase = 2.8;
  definition.default_mission.id = "mining-haul";
  definition.default_mission.type = domain::MissionType::kFollowRoute;
  definition.default_mission.route = definition.reference_route;
  definition.default_mission.speed_limit =
      definition.vehicle_constraints.maximum_speed;
  definition.default_behavior_profile = "MiningHaul";
  return definition;
}

}  // namespace sdc::scenario
