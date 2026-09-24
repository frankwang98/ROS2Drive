#pragma once

#include <string>
#include <vector>

#include "domain/autonomy_types.hpp"

namespace sdc::scenario {

// A lane centerline is the authoritative geometry shared by visualization,
// route materialization and mission execution.  Scenario adapters may render
// additional lanes, but must not duplicate the route's coordinates.
struct Lane {
  std::string id;
  std::vector<domain::Pose2D> centerline;
  double speed_limit{0.0};  // <= 0 means use the mission/scenario limit.
};

struct RoadNetwork {
  std::vector<Lane> lanes;

  const Lane* findLane(const std::string& id) const;
  std::vector<domain::Pose2D> materializeRoute(const std::vector<std::string>& lane_ids,
                                               std::string& reason) const;
};

}  // namespace sdc::scenario
