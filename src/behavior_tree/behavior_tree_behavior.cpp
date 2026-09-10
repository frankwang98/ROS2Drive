#include "behavior_tree/behavior_tree_behavior.hpp"

#include <algorithm>
#include <cmath>

namespace sdc {
BehaviorTreeBehavior::BehaviorTreeBehavior(std::string xml_path,
                                           std::string tree_id) {
  configuration_accepted_ = planner_.init(xml_path, tree_id);
}

behavior::BehaviorDecision BehaviorTreeBehavior::decide(
    const behavior::BehaviorInput& input) {
  double front_distance = 100.0;
  const double fx = std::cos(input.vehicle.pose.yaw);
  const double fy = std::sin(input.vehicle.pose.yaw);
  for (const auto& obstacle : input.obstacles) {
    const double dx = obstacle.pose.x - input.vehicle.pose.x;
    const double dy = obstacle.pose.y - input.vehicle.pose.y;
    if (dx * fx + dy * fy <= 0.0) continue;
    front_distance = std::min(front_distance,
                              std::max(0.0, std::hypot(dx, dy) - obstacle.radius));
  }
  const Action action = planner_.tick(front_distance);
  switch (action) {
    case Action::kStop: return {0.0, true, "STOP", "behavior_tree_stop"};
    case Action::kBrake: return {std::min(0.5, input.mission.speed_limit), false, "SLOW_DOWN", "obstacle_proximity"};
    case Action::kCruise: return {std::min(2.0, input.mission.speed_limit), false, "CRUISE", ""};
    case Action::kAccelerate: return {input.mission.speed_limit, false, "PROCEED", ""};
  }
  return {0.0, true, "STOP", "invalid_behavior_action"};
}
}  // namespace sdc
