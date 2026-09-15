#include "scenario/scenario_definition.hpp"

#include <cmath>

namespace sdc::scenario {

bool validate(const ScenarioDefinition& definition, std::string& reason) {
  if (definition.id.empty()) {
    reason = "scenario_id_empty";
    return false;
  }
  if (definition.default_behavior_profile.empty()) {
    reason = "behavior_profile_empty";
    return false;
  }
  if (definition.reference_route.size() < 2) {
    reason = "reference_route_too_short";
    return false;
  }
  if (!std::isfinite(definition.initial_pose.x) ||
      !std::isfinite(definition.initial_pose.y) ||
      !std::isfinite(definition.initial_pose.yaw)) {
    reason = "invalid_initial_pose";
    return false;
  }
  for (const auto& pose : definition.reference_route) {
    if (!std::isfinite(pose.x) || !std::isfinite(pose.y) ||
        !std::isfinite(pose.yaw)) {
      reason = "invalid_reference_route";
      return false;
    }
  }
  if (definition.default_mission.id.empty()) {
    reason = "default_mission_id_empty";
    return false;
  }
  const auto& limits = definition.vehicle_constraints;
  if (!std::isfinite(limits.maximum_speed) || limits.maximum_speed <= 0.0 ||
      !std::isfinite(limits.maximum_acceleration) ||
      limits.maximum_acceleration <= 0.0 ||
      !std::isfinite(limits.maximum_deceleration) ||
      limits.maximum_deceleration <= 0.0 ||
      !std::isfinite(limits.wheelbase) || limits.wheelbase <= 0.0) {
    reason = "invalid_vehicle_constraints";
    return false;
  }
  if (definition.default_mission.route.size() !=
      definition.reference_route.size()) {
    reason = "default_mission_route_mismatch";
    return false;
  }
  for (std::size_t i = 0; i < definition.reference_route.size(); ++i) {
    const auto& mission_pose = definition.default_mission.route[i];
    const auto& reference_pose = definition.reference_route[i];
    if (mission_pose.x != reference_pose.x ||
        mission_pose.y != reference_pose.y ||
        mission_pose.yaw != reference_pose.yaw) {
      reason = "default_mission_route_mismatch";
      return false;
    }
  }
  reason.clear();
  return true;
}

}  // namespace sdc::scenario
