#pragma once

#include <cstdint>
#include <string>

#include <builtin_interfaces/msg/time.hpp>
#include "runtime/vehicle_runtime.hpp"
#include "self_driving_car_demo/msg/control_command.hpp"
#include "self_driving_car_demo/msg/fault_array.hpp"
#include "self_driving_car_demo/msg/runtime_metrics.hpp"
#include "self_driving_car_demo/msg/runtime_status.hpp"
#include "self_driving_car_demo/msg/trajectory.hpp"

namespace sdc::ros {
struct MessageContext {
  std::string robot_id;
  std::string world_frame;
  std::string base_frame;
  builtin_interfaces::msg::Time stamp;
  uint64_t sequence{0};
};
class RuntimeMessageConverter {
 public:
  static self_driving_car_demo::msg::RuntimeStatus status(const runtime::VehicleRuntime& runtime,
                                                          const MessageContext& context,
                                                          bool localized);
  static self_driving_car_demo::msg::Trajectory trajectory(const runtime::VehicleRuntime& runtime,
                                                           const MessageContext& context);
  static self_driving_car_demo::msg::ControlCommand control(const runtime::VehicleRuntime& runtime,
                                                            const MessageContext& context);
  static self_driving_car_demo::msg::FaultArray faults(const runtime::VehicleRuntime& runtime,
                                                       const MessageContext& context);
  static self_driving_car_demo::msg::RuntimeMetrics metrics(const runtime::VehicleRuntime& runtime,
                                                            const MessageContext& context,
                                                            double loop_duration_ms,
                                                            double configured_loop_hz);
};
}  // namespace sdc::ros
