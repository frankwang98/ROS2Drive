#pragma once
#include <map>
#include "safety/fault_manager.hpp"
namespace sdc::safety {
struct SafetyConfig {
  double state_timeout_s{0.5};
  unsigned recovery_healthy_cycles{3};
  std::map<domain::FaultCode, domain::FaultAction> policies{
      {domain::FaultCode::kLocalizationLost, domain::FaultAction::kStop},
      {domain::FaultCode::kPlanningFailed, domain::FaultAction::kStop},
      {domain::FaultCode::kControlError, domain::FaultAction::kStop},
      {domain::FaultCode::kVehicleError, domain::FaultAction::kEmergencyStop},
      {domain::FaultCode::kInputTimeout, domain::FaultAction::kStop},
      {domain::FaultCode::kPerceptionTimeout, domain::FaultAction::kStop},
      {domain::FaultCode::kEmergencyStop, domain::FaultAction::kEmergencyStop}};
};
class SafetyManager {
 public:
  explicit SafetyManager(SafetyConfig config = {});
  domain::ControlCommand enforce(const domain::VehicleState&,
                                 bool planning_ok,
                                 double now_s,
                                 bool estop_requested,
                                 domain::ControlCommand desired,
                                 bool perception_ok = true,
                                 bool vehicle_ok = true,
                                 bool control_ok = true);
  // Clear a latched stop only after inputs remain healthy and the vehicle is
  // stationary. Releasing a hardware emergency-stop remains outside Runtime.
  bool acknowledgeRecovery();
  bool recoveryRequired() const {
    return latched_action_ >= domain::FaultAction::kStop;
  }
  bool recoveryReady() const {
    return recovery_ready_;
  }
  const FaultManager& faults() const {
    return faults_;
  }
  domain::FaultAction lastAction() const {
    return last_action_;
  }

 private:
  SafetyConfig config_;
  FaultManager faults_;
  domain::FaultAction actionFor(domain::FaultCode code) const;
  domain::FaultAction last_action_{domain::FaultAction::kReportOnly};
  domain::FaultAction latched_action_{domain::FaultAction::kReportOnly};
  unsigned healthy_cycles_{0};
  bool recovery_ready_{false};
};
}  // namespace sdc::safety
