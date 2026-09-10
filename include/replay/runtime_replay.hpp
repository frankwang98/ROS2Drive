#pragma once

#include <string>
#include <vector>

#include "runtime/vehicle_runtime.hpp"

namespace sdc::replay {
struct ReplayFrame {
  double timestamp_s{0.0};
  domain::VehicleState vehicle;
  std::vector<domain::Obstacle> obstacles;
  runtime::RuntimeOutput expected_output;
};
class RuntimeRecorder {
 public:
  void record(ReplayFrame frame);
  void clear() { frames_.clear(); }
  const std::vector<ReplayFrame>& frames() const { return frames_; }
  bool save(const std::string& path) const;
  bool load(const std::string& path);
 private:
  std::vector<ReplayFrame> frames_;
};
runtime::RuntimeOutput replayFrame(runtime::VehicleRuntime& runtime,
                                   const ReplayFrame& frame, double dt_s);
}  // namespace sdc::replay
