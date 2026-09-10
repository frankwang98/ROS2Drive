#pragma once

#include "domain/autonomy_types.hpp"

namespace sdc::planning {

class VelocityPlanner {
 public:
  struct Config {
    double maximum_lateral_acceleration{1.2};
    double maximum_acceleration{1.0};
    double maximum_deceleration{1.5};
    double minimum_curve_speed{0.3};
  };

  VelocityPlanner();
  explicit VelocityPlanner(Config config);
  void apply(double speed_limit, domain::Trajectory& trajectory) const;

 private:
  Config config_;
};

}  // namespace sdc::planning
