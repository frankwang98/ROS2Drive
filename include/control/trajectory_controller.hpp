#pragma once

#include "domain/autonomy_types.hpp"

namespace sdc::control {

struct ControllerInput {
  domain::VehicleState vehicle;
  domain::Trajectory trajectory;
  double dt_s{0.0};
};

class Controller {
 public:
  virtual ~Controller() = default;
  virtual domain::ControlCommand compute(const ControllerInput& input) = 0;
  virtual void reset() = 0;
};

class PurePursuitController final : public Controller {
 public:
  struct Config {
    double wheelbase{2.7};
    double minimum_lookahead{1.5};
    double lookahead_time{1.0};
    double maximum_steering{0.6};
  };

  PurePursuitController();
  explicit PurePursuitController(Config config);
  domain::ControlCommand compute(const ControllerInput& input) override;
  void reset() override {}

 private:
  Config config_;
};

}  // namespace sdc::control
