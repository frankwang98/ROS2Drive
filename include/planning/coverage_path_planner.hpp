#pragma once

#include "planning/reference_path_planner.hpp"

namespace sdc::planning {

class CoveragePathPlanner final : public Planner {
 public:
  struct Config {
    double min_x{-18.0}; double max_x{18.0};
    double min_y{-12.5}; double max_y{12.5};
    double row_spacing{5.0}; double headland_radius{2.5};
    double spacing{0.25}; double horizon{30.0}; double obstacle_margin{0.8};
  };
  CoveragePathPlanner();
  explicit CoveragePathPlanner(Config config);
  PlanningResult plan(const PlanningInput& input) override;
 private:
  std::vector<domain::Pose2D> build_rows() const;
  Config config_;
  ReferencePathPlanner dense_planner_;
};

}  // namespace sdc::planning
