#include "scenario/ring_scenario.hpp"

#include <cmath>
#include <stdexcept>
#include <utility>

namespace sdc::scenario {

ScenarioDefinition makeRingScenarioDefinition(double radius, int segments) {
  if (!std::isfinite(radius) || radius <= 0.0 || segments < 8)
    throw std::invalid_argument("ring scenario radius/segments are invalid");

  ScenarioDefinition definition;
  definition.id = "ring_demo";
  // CCW traffic keeps left of the double center line: inward lane radius.
  const double lane_offset = 1.5;
  const double route_radius = radius - lane_offset;
  definition.initial_pose = {route_radius, 0.0, M_PI_2};
  Lane lane;
  lane.id = "left_lane_loop";
  lane.speed_limit = definition.vehicle_constraints.maximum_speed;
  lane.centerline.reserve(static_cast<std::size_t>(segments) + 1);
  for (int i = 0; i <= segments; ++i) {
    const double angle = 2.0 * M_PI * i / segments;
    lane.centerline.push_back(
        {route_radius * std::cos(angle), route_radius * std::sin(angle), angle + M_PI_2});
  }
  definition.road_network.lanes.push_back(std::move(lane));
  definition.default_route_lane_ids = {"left_lane_loop"};
  definition.default_mission.id = "ring-demo";
  definition.default_mission.type = domain::MissionType::kFollowRoute;
  definition.default_mission.speed_limit = definition.vehicle_constraints.maximum_speed;
  // Deterministic benchmark obstacles.  Their positions are fixed so planner
  // and controller comparisons are repeatable across runs.
  definition.static_obstacles = {
      {"ring-benchmark-lane-1",
       {route_radius * std::cos(0.85), route_radius * std::sin(0.85), 0.0},
       0.8,
       false},
      {"ring-benchmark-lane-2",
       {route_radius * std::cos(3.00), route_radius * std::sin(3.00), 0.0},
       0.8,
       false},
      {"ring-benchmark-lane-3",
       {route_radius * std::cos(5.15), route_radius * std::sin(5.15), 0.0},
       0.8,
       false}};
  definition.default_behavior_profile = "RingDemo";
  return definition;
}

}  // namespace sdc::scenario
