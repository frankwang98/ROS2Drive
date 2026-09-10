#include "vehicle/simulated_vehicle.hpp"

#include <algorithm>
#include <cmath>

namespace sdc::vehicle {
domain::VehicleState SimulatedVehicle::state(double stamp_s) const {
  domain::VehicleState state;
  state.pose={model_.x(),model_.y(),model_.yaw()};
  state.velocity.linear=model_.speed();
  state.velocity.angular=model_.speed()*std::tan(model_.steer())/model_.wheelbase();
  state.steering_angle=model_.steer();
  state.stamp_s=stamp_s;
  state.localized=true;
  return state;
}
void SimulatedVehicle::apply(const domain::ControlCommand& command,double dt_s) {
  const double requested=(command.emergency_stop || command.brake>=0.99)?0.0:command.target_speed;
  const double speed=velocity_controller_.update(requested,model_.speed(),dt_s);
  model_.update(speed,command.steering_angle,dt_s);
}
bool SimulatedVehicle::healthy() const {
  return std::isfinite(model_.x()) && std::isfinite(model_.y()) &&
         std::isfinite(model_.yaw()) && std::isfinite(model_.speed());
}
void SimulatedVehicle::applyManual(double throttle,double steering,double dt_s) {
  const double target=std::clamp(throttle,-1.0,1.0)*4.0;
  const double speed=velocity_controller_.update(target,model_.speed(),dt_s);
  model_.update(speed,std::clamp(steering,-1.0,1.0)*0.55,dt_s);
}
void SimulatedVehicle::reset(double x,double y,double yaw) {
  model_.reset(x,y,yaw); velocity_controller_.reset();
}
}  // namespace sdc::vehicle
