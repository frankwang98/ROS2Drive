#include "runtime/vehicle_runtime.hpp"
#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace sdc::runtime {
VehicleRuntime::VehicleRuntime(std::unique_ptr<planning::Planner> planner)
    : VehicleRuntime(std::move(planner),
                     std::make_unique<control::PurePursuitController>(),
                     planning::VelocityPlanner(),
                     std::make_unique<behavior::PassthroughBehaviorManager>()) {}
VehicleRuntime::VehicleRuntime(
    std::unique_ptr<planning::Planner> planner,
    std::unique_ptr<control::Controller> controller,
    planning::VelocityPlanner velocity_planner,
    std::unique_ptr<behavior::BehaviorManager> behavior_manager)
    : planner_(std::move(planner)),
      controller_(std::move(controller)),
      behavior_manager_(std::move(behavior_manager)),
      velocity_planner_(std::move(velocity_planner)) {
  if(!planner_) throw std::invalid_argument("VehicleRuntime requires a planner");
  if(!controller_) throw std::invalid_argument("VehicleRuntime requires a controller");
  if(!behavior_manager_) throw std::invalid_argument("VehicleRuntime requires a behavior manager");
}
bool VehicleRuntime::setMission(domain::Mission mission, bool preempt, double now_s) {
  const auto policy=preempt?mission::SubmitPolicy::kPreemptActive:mission::SubmitPolicy::kRejectIfBusy;
  if(!missions_.submit(std::move(mission),policy,now_s)) return false;
  controller_->reset();
  const bool ok=missions_.start(now_s);
  output_.state=ok?domain::RuntimeState::kRunning:domain::RuntimeState::kReady;
  return ok;
}
bool VehicleRuntime::pause(){ if(!missions_.pause()) return false; output_.state=domain::RuntimeState::kPaused; return true; }
bool VehicleRuntime::resume(){ if(!missions_.resume()) return false; output_.state=domain::RuntimeState::kRunning; return true; }
bool VehicleRuntime::cancelMission(){ const bool ok=missions_.cancel(); if(ok){ controller_->reset(); output_.state=domain::RuntimeState::kStopped; } return ok; }
void VehicleRuntime::requestEmergencyStop(bool enabled){ estop_=enabled; }
bool VehicleRuntime::setBehaviorManager(std::unique_ptr<behavior::BehaviorManager> manager){ if(!manager) return false; behavior_manager_=std::move(manager); behavior_manager_->reset(); return true; }
bool VehicleRuntime::setPlanner(std::unique_ptr<planning::Planner> planner){ if(!planner) return false; planner_=std::move(planner); return true; }
bool VehicleRuntime::setController(std::unique_ptr<control::Controller> controller){ if(!controller) return false; controller_=std::move(controller); controller_->reset(); return true; }
void VehicleRuntime::setVelocityPlanner(planning::VelocityPlanner planner){ velocity_planner_=std::move(planner); }
void VehicleRuntime::setSafetyManager(safety::SafetyManager manager){ safety_=std::move(manager); }
void VehicleRuntime::updateVehicleState(domain::VehicleState state){ vehicle_=std::move(state); if(output_.state==domain::RuntimeState::kInit && vehicle_.localized) output_.state=domain::RuntimeState::kReady; }
void VehicleRuntime::updateObstacles(std::vector<domain::Obstacle> obstacles, double stamp_s){ obstacles_=std::move(obstacles); obstacles_stamp_s_=stamp_s; }
RuntimeOutput VehicleRuntime::step(double now, double dt){
  domain::ControlCommand desired;
  if(missions_.timedOut(now)) {
    missions_.fail("mission_timeout");
    controller_->reset();
  }
  const auto& mission=missions_.current();
  bool active=mission && mission->state==domain::MissionState::kActive;
  const bool stop_mission=active && mission->type==domain::MissionType::kStop;

  if(stop_mission) {
    output_.planning={};
    desired.target_speed=0.0;
    desired.brake=1.0;
    if(std::abs(vehicle_.velocity.linear)<0.05) {
      missions_.succeed("vehicle_stopped");
      active=false;
      output_.state=domain::RuntimeState::kStopped;
    }
  } else if(active){
    // Only scenarios that explicitly define an ordered load/dump/parking
    // sequence are staged missions.  Ordinary ring or point-to-point missions
    // keep the default zero indices and must not enter LOAD at startup.
    const bool staged_mission = mission->load_index < mission->dump_index &&
                                mission->dump_index < mission->parking_index &&
                                mission->parking_index < mission->route.size();
    const std::size_t stage_target_index =
        mission->stage == domain::MissionStage::kTransit ? mission->load_index :
        mission->stage == domain::MissionStage::kHaul ? mission->dump_index :
        mission->stage == domain::MissionStage::kReturn ? mission->parking_index :
        mission->route.size() - 1;
    const auto& stage_target = mission->route[stage_target_index];
    const double stage_distance = std::hypot(vehicle_.pose.x - stage_target.x,
                                             vehicle_.pose.y - stage_target.y);
    if (staged_mission && mission->stage == domain::MissionStage::kTransit &&
        stage_distance <= mission->goal_tolerance) {
      missions_.setStage(domain::MissionStage::kLoad, domain::PayloadState::kEmpty, now);
      output_.planning = {};
    } else if (staged_mission && mission->stage == domain::MissionStage::kLoad) {
      desired.brake = 1.0;
      if (now - mission->stage_started_at_s >= mission->work_hold_s)
        missions_.setStage(domain::MissionStage::kHaul, domain::PayloadState::kLoaded, now);
      output_.planning = {};
    } else if (staged_mission && mission->stage == domain::MissionStage::kHaul &&
               stage_distance <= mission->goal_tolerance) {
      missions_.setStage(domain::MissionStage::kDump, domain::PayloadState::kLoaded, now);
      desired.brake = 1.0;
      output_.planning = {};
    } else if (staged_mission && mission->stage == domain::MissionStage::kDump) {
      desired.brake = 1.0;
      if (now - mission->stage_started_at_s >= mission->work_hold_s)
        missions_.setStage(domain::MissionStage::kReturn, domain::PayloadState::kEmpty, now);
      output_.planning = {};
    } else if (staged_mission && mission->stage == domain::MissionStage::kReturn &&
               stage_distance <= mission->goal_tolerance) {
      missions_.succeed("parking_area_reached");
      controller_->reset();
      active = false;
      output_.planning = {};
      output_.state = domain::RuntimeState::kStopped;
    } else {
    behavior::BehaviorInput behavior_input{vehicle_, *mission, obstacles_};
    output_.behavior=behavior_manager_->decide(behavior_input);
    planning::PlanningInput in; in.vehicle=vehicle_; in.reference_path=mission->route;
    if (staged_mission) {
      std::size_t begin = mission->stage == domain::MissionStage::kHaul ? mission->load_index :
                          mission->stage == domain::MissionStage::kReturn ? mission->dump_index : 0;
      in.reference_path.assign(mission->route.begin() + begin,
                               mission->route.begin() + stage_target_index + 1);
    }
    if(mission->type==domain::MissionType::kNavigateTo && in.reference_path.size()==1)
      in.reference_path.insert(in.reference_path.begin(),vehicle_.pose);
    in.obstacles=obstacles_; in.speed_limit=std::min(mission->speed_limit,output_.behavior.speed_limit); in.now_s=now; output_.planning=planner_->plan(in);
    if(output_.planning.success && !output_.planning.trajectory.points.empty()) {
      velocity_planner_.apply(in.speed_limit, output_.planning.trajectory);
      control::ControllerInput controller_input;
      controller_input.vehicle=vehicle_;
      controller_input.trajectory=output_.planning.trajectory;
      controller_input.dt_s=dt;
      desired=controller_->compute(controller_input);
      if(output_.behavior.stop_required){ desired.target_speed=0.0; desired.brake=1.0; }
    }
    if(!staged_mission && !mission->route.empty()) {
      std::size_t nearest=0; double nearest_distance=std::numeric_limits<double>::max();
      for(std::size_t i=0;i<mission->route.size();++i) {
        const double distance=std::hypot(vehicle_.pose.x-mission->route[i].x,vehicle_.pose.y-mission->route[i].y);
        if(distance<nearest_distance){nearest_distance=distance;nearest=i;}
      }
      missions_.updateProgress(mission->route.size()>1?static_cast<double>(nearest)/(mission->route.size()-1):0.0);
      const auto& goal=mission->route.back();
      const double goal_distance=std::hypot(vehicle_.pose.x-goal.x,vehicle_.pose.y-goal.y);
      const bool closed=mission->route.size()>2 && std::hypot(mission->route.front().x-goal.x,mission->route.front().y-goal.y)<mission->goal_tolerance;
      if((mission->type==domain::MissionType::kFollowRoute &&
          mission->stage == domain::MissionStage::kHaul && !closed &&
          goal_distance <= mission->goal_tolerance)) {
        missions_.setStage(domain::MissionStage::kDump, domain::PayloadState::kLoaded);
        desired.target_speed = 0.0;
        desired.brake = 1.0;
      } else if((mission->type==domain::MissionType::kNavigateTo || !closed) && goal_distance<=mission->goal_tolerance) {
        missions_.succeed("goal_reached"); controller_->reset(); desired={}; active=false; output_.state=domain::RuntimeState::kStopped;
      }
    }
    }
  } else { output_.planning={}; }
  const bool vehicle_ok=std::isfinite(vehicle_.pose.x) && std::isfinite(vehicle_.pose.y) && std::isfinite(vehicle_.pose.yaw) && std::isfinite(vehicle_.velocity.linear);
  const bool perception_ok=obstacles_stamp_s_<=0.0 || (now>=obstacles_stamp_s_ && now-obstacles_stamp_s_<=perception_timeout_s_);
  const bool control_ok=std::isfinite(desired.target_speed) && std::isfinite(desired.steering_angle) && std::isfinite(desired.brake);
  output_.control=safety_.enforce(vehicle_,stop_mission || !active || output_.planning.success,now,estop_,desired,perception_ok,vehicle_ok,control_ok);
  if(estop_ || safety_.lastAction()==domain::FaultAction::kEmergencyStop) output_.state=domain::RuntimeState::kEstop;
  else if(safety_.lastAction()==domain::FaultAction::kStop) output_.state=domain::RuntimeState::kFault;
  else if(safety_.lastAction()==domain::FaultAction::kDegrade) output_.state=domain::RuntimeState::kDegraded;
  else if(active) output_.state=domain::RuntimeState::kRunning;
  else if(mission && mission->state==domain::MissionState::kPaused) output_.state=domain::RuntimeState::kPaused;
  else if(mission && (mission->state==domain::MissionState::kSucceeded || mission->state==domain::MissionState::kFailed || mission->state==domain::MissionState::kCanceled)) output_.state=domain::RuntimeState::kStopped;
  else output_.state=vehicle_.localized?domain::RuntimeState::kReady:domain::RuntimeState::kInit;
  return output_;
}
}  // namespace sdc::runtime
