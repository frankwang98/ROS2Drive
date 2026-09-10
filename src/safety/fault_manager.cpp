#include "safety/fault_manager.hpp"
#include <algorithm>
namespace sdc::safety {
void FaultManager::report(domain::Fault fault){ for(auto& f:faults_) if(f.code==fault.code){f=std::move(fault);return;} faults_.push_back(std::move(fault)); }
void FaultManager::clear(domain::FaultCode code){ faults_.erase(std::remove_if(faults_.begin(),faults_.end(),[code](const auto& f){return f.code==code;}),faults_.end()); }
bool FaultManager::hasError() const { return std::any_of(faults_.begin(),faults_.end(),[](const auto& f){return f.active && (f.severity==domain::FaultSeverity::kError || f.severity==domain::FaultSeverity::kFatal);}); }
bool FaultManager::hasFatal() const { return std::any_of(faults_.begin(),faults_.end(),[](const auto& f){return f.active && f.severity==domain::FaultSeverity::kFatal;}); }
bool FaultManager::hasWarning() const { return std::any_of(faults_.begin(),faults_.end(),[](const auto& f){return f.active && f.severity==domain::FaultSeverity::kWarning;}); }
}  // namespace sdc::safety
