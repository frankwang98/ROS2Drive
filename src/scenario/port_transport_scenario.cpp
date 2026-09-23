#include "scenario/port_transport_scenario.hpp"

namespace sdc::scenario {

ScenarioDefinition makePortTransportScenarioDefinition() {
  ScenarioDefinition definition;
  definition.id = "port_transport";
  definition.initial_pose = {-28.0, -14.0, 0.0};
  definition.reference_route = {
      {-28.0, -14.0, 0.0, 0.0}, {-22.0, -14.0, 0.0, 0.0},
      {-16.0, -14.0, 0.0, 0.0}, {-10.0, -14.0, 0.0, 0.0},
      {-8.5, -13.7, 0.2, 0.0},  {-7.3, -12.8, 0.6, 0.0},
      {-6.5, -11.5, 1.0, 0.0},  {-6.0, -9.0, 1.35, 0.0},
      {-6.0, -5.0, 1.57, 0.0},  {-5.6, -3.0, 1.30, 0.0},
      {-4.5, -1.3, 0.95, 0.0},  {-3.0, 0.0, 0.60, 0.0},
      {0.0, 1.0, 0.32, 0.0},    {4.0, 2.0, 0.25, 0.0},
      {8.0, 3.5, 0.45, 0.0},    {11.0, 5.5, 0.60, 0.0},
      // A wide quay turnaround.  The old route reversed at the berth in one
      // segment, which made a sharp 180-degree cusp that no forward vehicle
      // controller can track.  This connects to the return lane smoothly.
      {14.0, 7.0, 0.0, 0.0},    {14.8, 8.0, 0.90, 0.0},
      {15.0, 9.5, 1.47, 0.0},   {14.5, 11.0, 2.10, 0.0},
      {13.0, 12.0, 2.55, 0.0},  {11.0, 12.0, 3.14, 0.0},
      {9.5, 11.5, -2.80, 0.0},  {8.5, 10.0, -2.15, 0.0},
      {8.0, 8.0, -1.82, 0.0},   {8.0, 5.0, -1.57, 0.0},
      {8.0, 2.0, -1.57, 0.0},   {8.0, -2.0, -1.57, 0.0},
      {7.5, -4.0, -2.00, 0.0},
      {6.0, -5.5, -2.50, 0.0},  {4.0, -6.0, -3.00, 0.0},
      {0.0, -6.0, 3.14, 0.0},   {-4.0, -7.0, -2.90, 0.0},
      {-8.0, -9.0, -2.65, 0.0}, {-11.0, -11.0, -2.50, 0.0},
      {-14.0, -14.0, -2.30, 0.0}, {-16.0, -17.0, -2.10, 0.0},
      {-20.0, -18.0, 3.14, 0.0}, {-28.0, -18.0, 3.14, 0.0}};
  definition.static_obstacles.clear();
  definition.vehicle_constraints.maximum_speed = 1.2;
  definition.vehicle_constraints.maximum_acceleration = 0.6;
  definition.vehicle_constraints.maximum_deceleration = 1.0;
  definition.vehicle_constraints.wheelbase = 2.7;
  definition.default_mission.id = "port-transport";
  definition.default_mission.type = domain::MissionType::kFollowRoute;
  definition.default_mission.route = definition.reference_route;
  definition.default_mission.speed_limit = 1.2;
  definition.default_mission.goal_tolerance = 0.9;
  definition.default_mission.stage = domain::MissionStage::kTransit;
  definition.default_mission.payload = domain::PayloadState::kEmpty;
  definition.default_mission.load_index = 3;
  definition.default_mission.dump_index = 16;
  definition.default_mission.parking_index = 38;
  definition.default_mission.work_hold_s = 3.0;
  definition.default_behavior_profile = "PortTransport";
  return definition;
}

}  // namespace sdc::scenario
