#pragma once
#include <optional>
#include <deque>
#include <unordered_set>
#include "domain/autonomy_types.hpp"

namespace sdc::mission {
enum class SubmitPolicy { kRejectIfBusy, kPreemptActive };

class MissionManager {
 public:
  bool submit(domain::Mission mission,
              SubmitPolicy policy = SubmitPolicy::kRejectIfBusy,
              double now_s = 0.0);
  bool start(double now_s = 0.0);
  bool pause();
  bool resume();
  bool cancel(const std::string& reason = "canceled");
  bool succeed(const std::string& reason = "completed");
  bool fail(const std::string& reason);
  void updateProgress(double progress);
  bool timedOut(double now_s) const;
  bool busy() const;
  const std::string& lastError() const { return last_error_; }
  const std::optional<domain::Mission>& current() const { return current_; }
 private:
  static bool terminal(domain::MissionState state);
  static bool validate(const domain::Mission& mission, std::string& reason);
  std::optional<domain::Mission> current_;
  std::string last_error_;
  std::deque<std::string> recent_ids_;
  std::unordered_set<std::string> recent_id_set_;
  static constexpr std::size_t kRecentMissionLimit = 256;
};
}  // namespace sdc::mission
