#pragma once
#include <vector>
#include "domain/autonomy_types.hpp"

namespace sdc::safety {
class FaultManager {
 public:
  void report(domain::Fault fault);
  void clear(domain::FaultCode code);
  bool hasError() const;
  bool hasFatal() const;
  bool hasWarning() const;
  const std::vector<domain::Fault>& faults() const { return faults_; }
 private: std::vector<domain::Fault> faults_;
};
}  // namespace sdc::safety
