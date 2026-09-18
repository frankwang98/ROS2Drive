#include "scenario/mining_haul_scenario.hpp"

namespace sdc::scenario {

ScenarioDefinition makeMiningHaulScenarioDefinition() {
  ScenarioDefinition definition;
  definition.id = "mining_haul";
  definition.initial_pose = {-42.0, -22.0, 0.0};
  definition.reference_route = {
      {-42.0, -22.0, 0.0, 0.0},  {-38.0, -20.0, 0.0, 0.2},
      {-34.0, -18.0, 0.0, 0.5},   {-30.0, -16.0, 0.0, 1.0},
      {-24.0, -13.0, 0.1, 1.6},   {-16.0, -10.0, 0.2, 2.2},
      {-8.0, -7.0, 0.3, 2.8},      {0.0, -3.0, 0.4, 3.4},
      {8.0, 2.0, 0.5, 4.0},        {16.0, 8.0, 0.7, 4.8},
      {19.0, 11.0, 1.0, 5.2},      {20.0, 15.0, 1.3, 5.6},
      {18.0, 19.0, 1.7, 6.0},      {14.0, 22.0, 2.0, 6.4},
      {9.0, 24.0, 2.2, 6.8}};

  // Baseline mining route intentionally has no obstacles.  The first
  // acceptance step isolates route tracking and the LOAD/HAUL/DUMP/RETURN
  // mission flow; obstacle injection is added only after this path is stable.
  definition.static_obstacles.clear();

  definition.vehicle_constraints.maximum_speed = 1.5;
  definition.vehicle_constraints.maximum_acceleration = 0.6;
  definition.vehicle_constraints.maximum_deceleration = 1.0;
  definition.vehicle_constraints.wheelbase = 2.8;
  definition.default_mission.id = "mining-haul";
  definition.default_mission.type = domain::MissionType::kFollowRoute;
  definition.default_mission.route = definition.reference_route;
  definition.default_mission.speed_limit =
      definition.vehicle_constraints.maximum_speed;
  definition.default_mission.stage = domain::MissionStage::kTransit;
  definition.default_mission.payload = domain::PayloadState::kEmpty;
  definition.default_mission.load_index = 3;
  definition.default_mission.dump_index = 9;
  definition.default_mission.parking_index = 14;
  definition.default_mission.work_hold_s = 3.0;
  definition.default_behavior_profile = "MiningHaul";
  return definition;
}

}  // namespace sdc::scenario
