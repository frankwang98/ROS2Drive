#include "scenario/road_network.hpp"

#include <cmath>

namespace sdc::scenario {

const Lane* RoadNetwork::findLane(const std::string& id) const {
  for (const auto& lane : lanes) {
    if (lane.id == id)
      return &lane;
  }
  return nullptr;
}

std::vector<domain::Pose2D> RoadNetwork::materializeRoute(const std::vector<std::string>& lane_ids,
                                                          std::string& reason) const {
  std::vector<domain::Pose2D> route;
  for (const auto& id : lane_ids) {
    const Lane* lane = findLane(id);
    if (!lane) {
      reason = "route_lane_not_found:" + id;
      return {};
    }
    if (lane->centerline.size() < 2) {
      reason = "route_lane_too_short:" + id;
      return {};
    }
    const bool joins_previous =
        !route.empty() && std::hypot(route.back().x - lane->centerline.front().x,
                                     route.back().y - lane->centerline.front().y) < 1e-6;
    route.insert(
        route.end(), lane->centerline.begin() + (joins_previous ? 1 : 0), lane->centerline.end());
  }
  if (route.size() < 2)
    reason = "materialized_route_too_short";
  else
    reason.clear();
  return route;
}

}  // namespace sdc::scenario
