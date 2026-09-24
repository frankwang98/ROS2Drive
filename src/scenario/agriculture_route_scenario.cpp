#include "scenario/agriculture_route_scenario.hpp"

#include <utility>

namespace sdc::scenario {

ScenarioDefinition makeAgricultureRouteScenarioDefinition() {
  ScenarioDefinition definition;
  definition.id = "agriculture_route";

  // Six parallel work rows.  The short intermediate points at each headland
  // form a simple U-turn instead of asking the controller to turn in place.
  constexpr int kRows = 6;
  constexpr double x_left = -18.0;
  constexpr double x_right = 18.0;
  constexpr double first_y = -12.5;
  constexpr double row_spacing = 5.0;
  Lane lane;
  lane.id = "coverage_rows";
  lane.speed_limit = 1.0;
  for (int row = 0; row < kRows; ++row) {
    const double y = first_y + row * row_spacing;
    const bool left_to_right = (row % 2) == 0;
    if (left_to_right) {
      lane.centerline.push_back({x_left, y, 0.0, 0.0});
      lane.centerline.push_back({0.0, y, 0.0, 0.0});
      lane.centerline.push_back({x_right, y, 0.0, 0.0});
      if (row + 1 < kRows) {
        lane.centerline.push_back({20.0, y + 0.73, 0.4, 0.0});
        lane.centerline.push_back({20.5, y + 2.5, 1.57, 0.0});
        lane.centerline.push_back({20.0, y + 4.27, 2.7, 0.0});
        lane.centerline.push_back({x_right, y + row_spacing, 3.14159, 0.0});
      }
    } else {
      lane.centerline.push_back({x_right, y, 3.14159, 0.0});
      lane.centerline.push_back({0.0, y, 3.14159, 0.0});
      lane.centerline.push_back({x_left, y, 3.14159, 0.0});
      if (row + 1 < kRows) {
        lane.centerline.push_back({-20.0, y + 0.73, 2.7, 0.0});
        lane.centerline.push_back({-20.5, y + 2.5, 1.57, 0.0});
        lane.centerline.push_back({-20.0, y + 4.27, 0.4, 0.0});
        lane.centerline.push_back({x_left, y + row_spacing, 0.0, 0.0});
      }
    }
  }
  // Leave the final work row through the headland and park above the field.
  lane.centerline.push_back({-16.0, 13.0, 0.4, 0.0});
  lane.centerline.push_back({-8.0, 15.0, 0.4, 0.0});
  lane.centerline.push_back({0.0, 15.0, 0.0, 0.0});
  definition.initial_pose = lane.centerline.front();
  definition.road_network.lanes.push_back(std::move(lane));
  definition.default_route_lane_ids = {"coverage_rows"};
  definition.static_obstacles.clear();
  definition.vehicle_constraints.maximum_speed = 1.0;
  definition.vehicle_constraints.maximum_acceleration = 0.5;
  definition.vehicle_constraints.maximum_deceleration = 1.0;
  definition.vehicle_constraints.wheelbase = 2.2;
  definition.default_mission.id = "agriculture-coverage";
  definition.default_mission.type = domain::MissionType::kFollowRoute;
  definition.default_mission.speed_limit = 1.0;
  definition.default_mission.goal_tolerance = 0.9;
  // Must match the tree ID in config/behavior_trees/scene_driving.xml.
  definition.default_behavior_profile = "AgricultureRoute";
  return definition;
}

}  // namespace sdc::scenario
