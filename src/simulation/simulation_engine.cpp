#include "simulation/simulation_engine.hpp"

#include <utility>

namespace sdc::simulation {

SimulationEngine::SimulationEngine(std::unique_ptr<planning::Planner> planner)
    : runtime_(std::move(planner)) {}
SimulationEngine::SimulationEngine(
    std::unique_ptr<planning::Planner> planner,
    std::unique_ptr<behavior::BehaviorManager> behavior_manager)
    : runtime_(std::move(planner),
               std::make_unique<control::PurePursuitController>(),
               planning::VelocityPlanner(), std::move(behavior_manager)) {}

bool SimulationEngine::setMission(domain::Mission mission, bool preempt) {
  return runtime_.setMission(std::move(mission), preempt,
                             simulation_time_s_ + 1.0);
}

void SimulationEngine::setObstacles(std::vector<domain::Obstacle> obstacles) {
  obstacles_ = std::move(obstacles);
}

runtime::RuntimeOutput SimulationEngine::step(double dt_s) {
  simulation_time_s_ += dt_s;
  domain::VehicleState state=vehicle_.state(simulation_time_s_+1.0);
  state.localized = fault_injection_.localization_available;
  runtime_.updateVehicleState(state);
  if (fault_injection_.perception_available)
    runtime_.updateObstacles(obstacles_, state.stamp_s);
  auto output = runtime_.step(state.stamp_s, dt_s);
  if(recording_enabled_) recorder_.record({state.stamp_s,state,obstacles_,output});
  vehicle_.apply(output.control,dt_s);
  return output;
}

void SimulationEngine::manualStep(double throttle, double steering,
                                  double dt_s) {
  simulation_time_s_ += dt_s;
  vehicle_.applyManual(throttle,steering,dt_s);
}

void SimulationEngine::reset(double x, double y, double yaw) {
  vehicle_.reset(x, y, yaw);
  simulation_time_s_ = 0.0;
}
bool SimulationEngine::pause() { return runtime_.pause(); }
bool SimulationEngine::resume() { return runtime_.resume(); }
bool SimulationEngine::cancelMission() { return runtime_.cancelMission(); }
void SimulationEngine::requestEmergencyStop(bool enabled) {
  runtime_.requestEmergencyStop(enabled);
}
bool SimulationEngine::setBehaviorManager(std::unique_ptr<behavior::BehaviorManager> manager) {
  return runtime_.setBehaviorManager(std::move(manager));
}
bool SimulationEngine::setPlanner(std::unique_ptr<planning::Planner> planner) { return runtime_.setPlanner(std::move(planner)); }
bool SimulationEngine::setController(std::unique_ptr<control::Controller> controller) { return runtime_.setController(std::move(controller)); }
void SimulationEngine::setVelocityPlanner(planning::VelocityPlanner planner) { runtime_.setVelocityPlanner(std::move(planner)); }
void SimulationEngine::setSafetyManager(safety::SafetyManager manager) { runtime_.setSafetyManager(std::move(manager)); }

}  // namespace sdc::simulation
