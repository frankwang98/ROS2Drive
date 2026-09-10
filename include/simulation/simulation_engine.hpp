#pragma once

#include <memory>
#include <vector>

#include "runtime/vehicle_runtime.hpp"
#include "vehicle/simulated_vehicle.hpp"
#include "replay/runtime_replay.hpp"

namespace sdc::simulation {

struct FaultInjection {
  bool localization_available{true};
  bool perception_available{true};
};

class SimulationEngine {
 public:
  explicit SimulationEngine(std::unique_ptr<planning::Planner> planner);
  SimulationEngine(std::unique_ptr<planning::Planner> planner,
                   std::unique_ptr<behavior::BehaviorManager> behavior_manager);
  bool setMission(domain::Mission mission, bool preempt = true);
  void setObstacles(std::vector<domain::Obstacle> obstacles);
  runtime::RuntimeOutput step(double dt_s);
  void manualStep(double throttle, double steering, double dt_s);
  void reset(double x, double y, double yaw);
  bool pause();
  bool resume();
  bool cancelMission();
  void requestEmergencyStop(bool enabled);
  bool acknowledgeSafetyRecovery() { return runtime_.acknowledgeSafetyRecovery(); }
  void setFaultInjection(FaultInjection injection) { fault_injection_ = injection; }
  void enableRecording(bool enabled) { recording_enabled_ = enabled; }
  replay::RuntimeRecorder& recorder() { return recorder_; }
  const replay::RuntimeRecorder& recorder() const { return recorder_; }
  bool setBehaviorManager(std::unique_ptr<behavior::BehaviorManager> manager);
  bool setPlanner(std::unique_ptr<planning::Planner> planner);
  bool setController(std::unique_ptr<control::Controller> controller);
  void setVelocityPlanner(planning::VelocityPlanner planner);
  void setSafetyManager(safety::SafetyManager manager);

  const AckermannModel& vehicle() const { return vehicle_.model(); }
  const runtime::VehicleRuntime& runtime() const { return runtime_; }
  double simulationTime() const { return simulation_time_s_; }

 private:
  vehicle::SimulatedVehicle vehicle_;
  runtime::VehicleRuntime runtime_;
  std::vector<domain::Obstacle> obstacles_;
  double simulation_time_s_{0.0};
  FaultInjection fault_injection_;
  bool recording_enabled_{false};
  replay::RuntimeRecorder recorder_;
};

}  // namespace sdc::simulation
