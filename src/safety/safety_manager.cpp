#include "safety/safety_manager.hpp"
#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>
namespace sdc::safety {
SafetyManager::SafetyManager(SafetyConfig config) : config_(std::move(config)) {
  if(config_.state_timeout_s<=0.0 || config_.recovery_healthy_cycles==0)
    throw std::invalid_argument("SafetyConfig timeout and recovery cycles must be positive");
}
domain::FaultAction SafetyManager::actionFor(domain::FaultCode code) const {
  const auto policy=config_.policies.find(code);
  return policy==config_.policies.end()?domain::FaultAction::kStop:policy->second;
}
bool SafetyManager::acknowledgeRecovery() {
  if(!recovery_ready_) return false;
  latched_action_=domain::FaultAction::kReportOnly;
  healthy_cycles_=0;
  recovery_ready_=false;
  return true;
}
domain::ControlCommand SafetyManager::enforce(const domain::VehicleState& state, bool planning_ok, double now, bool estop, domain::ControlCommand command, bool perception_ok, bool vehicle_ok, bool control_ok) {
  faults_.clear(domain::FaultCode::kLocalizationLost); faults_.clear(domain::FaultCode::kPlanningFailed); faults_.clear(domain::FaultCode::kInputTimeout); faults_.clear(domain::FaultCode::kPerceptionTimeout); faults_.clear(domain::FaultCode::kVehicleError); faults_.clear(domain::FaultCode::kControlError); faults_.clear(domain::FaultCode::kEmergencyStop);
  if(!state.localized) faults_.report({domain::FaultCode::kLocalizationLost,domain::FaultSeverity::kError,"localization unavailable",now});
  if(state.stamp_s<=0.0 || now-state.stamp_s>config_.state_timeout_s) faults_.report({domain::FaultCode::kInputTimeout,domain::FaultSeverity::kError,"vehicle state timeout",now});
  if(!planning_ok) faults_.report({domain::FaultCode::kPlanningFailed,domain::FaultSeverity::kError,"planner returned no valid trajectory",now});
  if(!perception_ok) faults_.report({domain::FaultCode::kPerceptionTimeout,domain::FaultSeverity::kError,"perception input timeout",now});
  if(!vehicle_ok) faults_.report({domain::FaultCode::kVehicleError,domain::FaultSeverity::kFatal,"invalid vehicle state",now});
  if(!control_ok) faults_.report({domain::FaultCode::kControlError,domain::FaultSeverity::kError,"invalid control output",now});
  if(estop) faults_.report({domain::FaultCode::kEmergencyStop,domain::FaultSeverity::kFatal,"emergency stop requested",now});
  domain::FaultAction strongest=domain::FaultAction::kReportOnly;
  for(const auto& fault:faults_.faults()) if(fault.active)
    strongest=static_cast<domain::FaultAction>(std::max(static_cast<int>(strongest),static_cast<int>(actionFor(fault.code))));
  if(strongest>=domain::FaultAction::kStop)
    latched_action_=static_cast<domain::FaultAction>(std::max(static_cast<int>(latched_action_),static_cast<int>(strongest)));
  const bool currently_healthy=strongest<domain::FaultAction::kStop && !estop && std::abs(state.velocity.linear)<0.05;
  healthy_cycles_=currently_healthy?healthy_cycles_+1:0;
  recovery_ready_=latched_action_>=domain::FaultAction::kStop && healthy_cycles_>=config_.recovery_healthy_cycles;
  strongest=static_cast<domain::FaultAction>(std::max(static_cast<int>(strongest),static_cast<int>(latched_action_)));
  last_action_=strongest;
  if(strongest==domain::FaultAction::kDegrade) command.target_speed=std::min(command.target_speed,0.5);
  if(strongest==domain::FaultAction::kStop || strongest==domain::FaultAction::kEmergencyStop){ command.target_speed=0.0; command.brake=1.0; }
  command.emergency_stop=strongest==domain::FaultAction::kEmergencyStop;
  return command;
}
}  // namespace sdc::safety
