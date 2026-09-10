#pragma once

#include "domain/autonomy_types.hpp"

namespace sdc::vehicle {
class VehicleInterface {
 public:
  virtual ~VehicleInterface() = default;
  virtual domain::VehicleState state(double stamp_s) const = 0;
  virtual void apply(const domain::ControlCommand& command, double dt_s) = 0;
  virtual bool healthy() const = 0;
};
}  // namespace sdc::vehicle
