#pragma once
#include <memory>
#include <vector>
#include "behavior/behavior_manager.hpp"
#include "control/trajectory_controller.hpp"
#include "mission/mission_manager.hpp"
#include "planning/planner.hpp"
#include "planning/velocity_planner.hpp"
#include "safety/safety_manager.hpp"

namespace sdc::runtime {
struct RuntimeOutput { domain::RuntimeState state{domain::RuntimeState::kInit}; behavior::BehaviorDecision behavior; planning::PlanningResult planning; domain::ControlCommand control; };

class VehicleRuntime {
 public:
  explicit VehicleRuntime(std::unique_ptr<planning::Planner> planner);
  VehicleRuntime(std::unique_ptr<planning::Planner> planner,
                 std::unique_ptr<control::Controller> controller,
                 planning::VelocityPlanner velocity_planner,
                 std::unique_ptr<behavior::BehaviorManager> behavior_manager =
                     std::make_unique<behavior::PassthroughBehaviorManager>());
  bool setMission(domain::Mission mission, bool preempt = false,
                  double now_s = 0.0);
  bool pause();
  bool resume();
  bool cancelMission();
  void requestEmergencyStop(bool enabled);
  bool acknowledgeSafetyRecovery() { return safety_.acknowledgeRecovery(); }
  bool safetyRecoveryRequired() const { return safety_.recoveryRequired(); }
  bool safetyRecoveryReady() const { return safety_.recoveryReady(); }
  bool setBehaviorManager(std::unique_ptr<behavior::BehaviorManager> manager);
  bool setPlanner(std::unique_ptr<planning::Planner> planner);
  bool setController(std::unique_ptr<control::Controller> controller);
  void setVelocityPlanner(planning::VelocityPlanner planner);
  void setSafetyManager(safety::SafetyManager manager);
  void updateVehicleState(domain::VehicleState state);
  void updateObstacles(std::vector<domain::Obstacle> obstacles,
                       double stamp_s = 0.0);
  RuntimeOutput step(double now_s, double dt_s);
  const RuntimeOutput& output() const { return output_; }
  const mission::MissionManager& missions() const { return missions_; }
  const safety::FaultManager& faults() const { return safety_.faults(); }
  const safety::SafetyManager& safety() const { return safety_; }
 private:
  std::unique_ptr<planning::Planner> planner_;
  std::unique_ptr<control::Controller> controller_;
  std::unique_ptr<behavior::BehaviorManager> behavior_manager_;
  planning::VelocityPlanner velocity_planner_;
  mission::MissionManager missions_;
  safety::SafetyManager safety_;
  domain::VehicleState vehicle_;
  std::vector<domain::Obstacle> obstacles_;
  double obstacles_stamp_s_{0.0};
  double perception_timeout_s_{0.5};
  RuntimeOutput output_;
  bool estop_{false};
};
}  // namespace sdc::runtime
