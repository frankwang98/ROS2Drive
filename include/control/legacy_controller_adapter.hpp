#pragma once

#include "control/trajectory_controller.hpp"
#include "control/steering_controller.hpp"
#include "control/lqr_controller.hpp"
#include "control/mpc_controller.hpp"

namespace sdc::control {

/// Adapter for the previously standalone lateral controllers.  It consumes
/// the same dense Runtime trajectory as PurePursuitController.
class LegacyControllerAdapter final : public Controller {
 public:
  enum class Type { kStanley, kLqr, kMpc };
  LegacyControllerAdapter(Type type, double wheelbase, double max_steering, double dt_s);
  domain::ControlCommand compute(const ControllerInput& input) override;
  void reset() override;

 private:
  Type type_;
  SteeringController stanley_;
  LqrController lqr_;
  MpcController mpc_;
};

}  // namespace sdc::control
