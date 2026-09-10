#pragma once

#include "control/velocity_controller.hpp"
#include "model/ackermann_model.hpp"
#include "vehicle/vehicle_interface.hpp"

namespace sdc::vehicle {
class SimulatedVehicle final : public VehicleInterface {
 public:
  domain::VehicleState state(double stamp_s) const override;
  void apply(const domain::ControlCommand& command, double dt_s) override;
  bool healthy() const override;
  void applyManual(double throttle, double steering, double dt_s);
  void reset(double x, double y, double yaw);
  const AckermannModel& model() const { return model_; }
 private:
  AckermannModel model_;
  VelocityController velocity_controller_;
};
}  // namespace sdc::vehicle
