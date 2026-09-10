#include "mission/mission_manager.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

namespace sdc::mission {
bool MissionManager::terminal(domain::MissionState state) {
  return state == domain::MissionState::kSucceeded || state == domain::MissionState::kFailed || state == domain::MissionState::kCanceled;
}
bool MissionManager::validate(const domain::Mission& mission, std::string& reason) {
  if (mission.id.empty()) { reason = "mission_id_empty"; return false; }
  if (!std::isfinite(mission.speed_limit) || mission.speed_limit < 0.0) { reason = "invalid_speed_limit"; return false; }
  if (!std::isfinite(mission.goal_tolerance) || mission.goal_tolerance <= 0.0) { reason = "invalid_goal_tolerance"; return false; }
  if (mission.type == domain::MissionType::kNavigateTo && mission.route.empty()) { reason = "navigate_goal_missing"; return false; }
  if (mission.type == domain::MissionType::kFollowRoute && mission.route.size() < 2) { reason = "follow_route_too_short"; return false; }
  reason.clear(); return true;
}
bool MissionManager::submit(domain::Mission mission, SubmitPolicy policy, double now_s) {
  if (!validate(mission, last_error_)) return false;
  if (recent_id_set_.count(mission.id) != 0) { last_error_ = "duplicate_mission_id"; return false; }
  if (busy() && policy == SubmitPolicy::kRejectIfBusy) { last_error_ = "mission_busy"; return false; }
  if (busy()) cancel("preempted_by_new_mission");
  mission.state=domain::MissionState::kPending; mission.submitted_at_s=now_s; mission.started_at_s=0.0; mission.progress=0.0; mission.result_reason.clear();
  recent_ids_.push_back(mission.id); recent_id_set_.insert(mission.id);
  if(recent_ids_.size()>kRecentMissionLimit){ recent_id_set_.erase(recent_ids_.front()); recent_ids_.pop_front(); }
  current_=std::move(mission); last_error_.clear(); return true;
}
bool MissionManager::start(double now_s) { if(!current_ || current_->state!=domain::MissionState::kPending) return false; current_->state=domain::MissionState::kActive; current_->started_at_s=now_s; return true; }
bool MissionManager::pause() { if(!current_ || current_->state!=domain::MissionState::kActive) return false; current_->state=domain::MissionState::kPaused; return true; }
bool MissionManager::resume() { if(!current_ || current_->state!=domain::MissionState::kPaused) return false; current_->state=domain::MissionState::kActive; return true; }
bool MissionManager::cancel(const std::string& reason) { if(!current_ || terminal(current_->state)) return false; current_->state=domain::MissionState::kCanceled; current_->result_reason=reason; return true; }
bool MissionManager::succeed(const std::string& reason) { if(!current_ || terminal(current_->state)) return false; current_->state=domain::MissionState::kSucceeded; current_->progress=1.0; current_->result_reason=reason; return true; }
bool MissionManager::fail(const std::string& reason) { if(!current_ || terminal(current_->state)) return false; current_->state=domain::MissionState::kFailed; current_->result_reason=reason; return true; }
void MissionManager::updateProgress(double progress) { if(current_ && !terminal(current_->state)) current_->progress=std::clamp(progress,0.0,1.0); }
bool MissionManager::timedOut(double now_s) const { return current_ && current_->state==domain::MissionState::kActive && current_->timeout_s>0.0 && current_->started_at_s>0.0 && now_s-current_->started_at_s>current_->timeout_s; }
bool MissionManager::busy() const { return current_ && !terminal(current_->state); }
}  // namespace sdc::mission
