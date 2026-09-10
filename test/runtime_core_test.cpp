#include <gtest/gtest.h>

#include <cmath>
#include <memory>
#include <stdexcept>

#include "mission/mission_manager.hpp"
#include "planning/reference_path_planner.hpp"
#include "runtime/vehicle_runtime.hpp"
#include "safety/safety_manager.hpp"
#include "simulation/simulation_engine.hpp"

namespace {
using sdc::domain::Mission;
using sdc::domain::MissionState;
using sdc::domain::MissionType;
using sdc::domain::Pose2D;

class FailingPlanner final : public sdc::planning::Planner {
 public:
  sdc::planning::PlanningResult plan(
      const sdc::planning::PlanningInput&) override {
    return {{}, false, "injected_planning_failure"};
  }
};

TEST(MissionManager, ValidatesLifecycleAndPreemption) {
  sdc::mission::MissionManager manager;
  Mission first;
  first.id = "route-1";
  first.type = MissionType::kFollowRoute;
  first.route = {{0.0, 0.0, 0.0}, {5.0, 0.0, 0.0}};
  EXPECT_TRUE(manager.submit(first, sdc::mission::SubmitPolicy::kRejectIfBusy, 1.0));
  EXPECT_TRUE(manager.start(2.0));
  EXPECT_EQ(manager.current()->state, MissionState::kActive);

  Mission second = first;
  second.id = "route-2";
  EXPECT_FALSE(manager.submit(second));
  EXPECT_EQ(manager.lastError(), "mission_busy");
  EXPECT_TRUE(manager.submit(second, sdc::mission::SubmitPolicy::kPreemptActive, 3.0));
  EXPECT_EQ(manager.current()->id, "route-2");
  EXPECT_FALSE(manager.submit(second, sdc::mission::SubmitPolicy::kPreemptActive, 4.0));
  EXPECT_EQ(manager.lastError(), "duplicate_mission_id");
}

TEST(MissionManager, DetectsTimeoutAndKeepsResult) {
  sdc::mission::MissionManager manager;
  Mission mission;
  mission.id = "timed";
  mission.type = MissionType::kStop;
  mission.timeout_s = 2.0;
  ASSERT_TRUE(manager.submit(mission, sdc::mission::SubmitPolicy::kRejectIfBusy, 1.0));
  ASSERT_TRUE(manager.start(1.0));
  EXPECT_FALSE(manager.timedOut(2.9));
  EXPECT_TRUE(manager.timedOut(3.1));
  EXPECT_TRUE(manager.fail("mission_timeout"));
  EXPECT_EQ(manager.current()->state, MissionState::kFailed);
  EXPECT_EQ(manager.current()->result_reason, "mission_timeout");
}

TEST(ReferencePathPlanner, ProducesDenseTrajectoryForNonRingRoute) {
  sdc::planning::ReferencePathPlanner planner;
  sdc::planning::PlanningInput input;
  input.vehicle.localized = true;
  input.vehicle.pose = {0.0, 0.0, 0.0};
  input.reference_path = {{0.0, 0.0, 0.0}, {4.0, 0.0, 0.0},
                          {4.0, 4.0, 1.57}};
  input.speed_limit = 2.0;
  input.now_s = 1.0;
  const auto result = planner.plan(input);
  ASSERT_TRUE(result.success);
  EXPECT_GT(result.trajectory.points.size(), 20u);
  EXPECT_NEAR(result.trajectory.points.back().pose.x, 4.0, 0.3);
  EXPECT_NEAR(result.trajectory.points.back().pose.y, 4.0, 0.3);
}

TEST(ReferencePathPlanner, AvoidsObstacleWithoutRingGeometry) {
  sdc::planning::ReferencePathPlanner planner;
  sdc::planning::PlanningInput input;
  input.vehicle.localized = true;
  input.vehicle.pose = {0.0, 0.0, 0.0};
  input.reference_path = {{0.0, 0.0, 0.0}, {12.0, 0.0, 0.0}};
  sdc::domain::Obstacle obstacle;
  obstacle.id = "moving-worker";
  obstacle.pose = {4.0, 0.0, 0.0};
  obstacle.radius = 0.5;
  obstacle.dynamic = true;
  input.obstacles.push_back(obstacle);
  input.speed_limit = 1.5;
  const auto result = planner.plan(input);
  ASSERT_TRUE(result.success);
  bool shifted = false;
  for (const auto& point : result.trajectory.points)
    shifted = shifted || std::abs(point.pose.y) > 0.5;
  EXPECT_TRUE(shifted);
}

TEST(SafetyManager, StopsForTimeoutPlanningFailureAndEstop) {
  sdc::safety::SafetyManager safety;
  sdc::domain::VehicleState state;
  state.localized = true;
  state.stamp_s = 1.0;
  sdc::domain::ControlCommand desired;
  desired.target_speed = 2.0;

  auto command = safety.enforce(state, true, 2.0, false, desired);
  EXPECT_DOUBLE_EQ(command.target_speed, 0.0);
  EXPECT_DOUBLE_EQ(command.brake, 1.0);

  state.stamp_s = 2.0;
  command = safety.enforce(state, false, 2.1, false, desired);
  EXPECT_DOUBLE_EQ(command.target_speed, 0.0);

  command = safety.enforce(state, true, 2.1, true, desired);
  EXPECT_TRUE(command.emergency_stop);
}

TEST(SafetyManager, AppliesConfiguredDegradePolicy) {
  sdc::safety::SafetyConfig config;
  config.policies[sdc::domain::FaultCode::kLocalizationLost] =
      sdc::domain::FaultAction::kDegrade;
  sdc::safety::SafetyManager safety(config);
  sdc::domain::VehicleState state;
  state.localized = false;
  state.stamp_s = 2.0;
  sdc::domain::ControlCommand desired;
  desired.target_speed = 2.0;
  const auto command = safety.enforce(state, true, 2.1, false, desired);
  EXPECT_DOUBLE_EQ(command.target_speed, 0.5);
  EXPECT_FALSE(command.emergency_stop);
  EXPECT_EQ(safety.lastAction(), sdc::domain::FaultAction::kDegrade);
}

TEST(SafetyManager, RejectsUnsafeConfiguration) {
  sdc::safety::SafetyConfig config;
  config.recovery_healthy_cycles = 0;
  EXPECT_THROW(sdc::safety::SafetyManager safety(config), std::invalid_argument);
}

TEST(SafetyManager, StopsForStalePerceptionAndInvalidControl) {
  sdc::safety::SafetyManager safety;
  sdc::domain::VehicleState state;
  state.localized = true;
  state.stamp_s = 2.0;
  sdc::domain::ControlCommand desired;
  desired.target_speed = 1.0;
  auto command = safety.enforce(state, true, 2.1, false, desired,
                                false, true, true);
  EXPECT_DOUBLE_EQ(command.target_speed, 0.0);
  EXPECT_DOUBLE_EQ(command.brake, 1.0);
  command = safety.enforce(state, true, 2.1, false, desired,
                           true, true, false);
  EXPECT_DOUBLE_EQ(command.target_speed, 0.0);
}

TEST(SafetyManager, StopFaultRequiresHealthyAcknowledgedRecovery) {
  sdc::safety::SafetyConfig config;
  config.recovery_healthy_cycles = 2;
  sdc::safety::SafetyManager safety(config);
  sdc::domain::VehicleState state;
  state.localized = false;
  state.stamp_s = 1.0;
  sdc::domain::ControlCommand desired;
  desired.target_speed = 1.0;
  EXPECT_DOUBLE_EQ(safety.enforce(state, true, 1.1, false, desired).target_speed,
                   0.0);
  EXPECT_TRUE(safety.recoveryRequired());
  EXPECT_FALSE(safety.acknowledgeRecovery());

  state.localized = true;
  state.velocity.linear = 0.0;
  state.stamp_s = 1.2;
  safety.enforce(state, true, 1.2, false, desired);
  safety.enforce(state, true, 1.3, false, desired);
  EXPECT_TRUE(safety.recoveryReady());
  EXPECT_TRUE(safety.acknowledgeRecovery());
  EXPECT_FALSE(safety.recoveryRequired());
  EXPECT_DOUBLE_EQ(safety.enforce(state, true, 1.3, false, desired).target_speed,
                   1.0);
}

TEST(VehicleRuntime, CompletesStopMission) {
  sdc::runtime::VehicleRuntime runtime(
      std::make_unique<sdc::planning::ReferencePathPlanner>());
  sdc::domain::VehicleState state;
  state.localized = true;
  state.stamp_s = 1.0;
  runtime.updateVehicleState(state);
  Mission mission;
  mission.id = "stop";
  mission.type = MissionType::kStop;
  ASSERT_TRUE(runtime.setMission(mission, false, 1.0));
  const auto output = runtime.step(1.1, 0.05);
  EXPECT_EQ(output.state, sdc::domain::RuntimeState::kStopped);
  EXPECT_EQ(runtime.missions().current()->state, MissionState::kSucceeded);
  EXPECT_EQ(runtime.missions().current()->result_reason, "vehicle_stopped");
}

TEST(VehicleRuntime, CompletesNavigateToInsideGoalTolerance) {
  sdc::runtime::VehicleRuntime runtime(
      std::make_unique<sdc::planning::ReferencePathPlanner>());
  sdc::domain::VehicleState state;
  state.localized = true;
  state.pose = {0.0, 0.0, 0.0};
  state.stamp_s = 1.0;
  runtime.updateVehicleState(state);
  Mission mission;
  mission.id = "navigate";
  mission.type = MissionType::kNavigateTo;
  mission.route = {{0.2, 0.0, 0.0}};
  mission.goal_tolerance = 0.5;
  ASSERT_TRUE(runtime.setMission(mission, false, 1.0));
  runtime.step(1.1, 0.05);
  EXPECT_EQ(runtime.missions().current()->state, MissionState::kSucceeded);
}

TEST(SimulationEngine, InjectedLocalizationLossStopsVehicle) {
  sdc::simulation::SimulationEngine simulation(
      std::make_unique<sdc::planning::ReferencePathPlanner>());
  simulation.reset(0.0, 0.0, 0.0);
  Mission mission;
  mission.id = "route";
  mission.type = MissionType::kFollowRoute;
  mission.route = {{0.0, 0.0, 0.0}, {10.0, 0.0, 0.0}};
  ASSERT_TRUE(simulation.setMission(mission));
  simulation.setObstacles({});
  simulation.setFaultInjection({false, true});
  const auto output = simulation.step(0.05);
  EXPECT_EQ(output.state, sdc::domain::RuntimeState::kFault);
  EXPECT_DOUBLE_EQ(output.control.target_speed, 0.0);
  EXPECT_DOUBLE_EQ(output.control.brake, 1.0);
}

TEST(VehicleRuntime, PlanningFailureTransitionsToFault) {
  sdc::runtime::VehicleRuntime runtime(std::make_unique<FailingPlanner>());
  sdc::domain::VehicleState state;
  state.localized = true;
  state.stamp_s = 1.0;
  runtime.updateVehicleState(state);
  Mission mission;
  mission.id = "failure";
  mission.type = MissionType::kFollowRoute;
  mission.route = {{0.0, 0.0, 0.0}, {5.0, 0.0, 0.0}};
  ASSERT_TRUE(runtime.setMission(mission, false, 1.0));
  const auto output = runtime.step(1.1, 0.05);
  EXPECT_EQ(output.state, sdc::domain::RuntimeState::kFault);
  EXPECT_EQ(output.planning.reason, "injected_planning_failure");
  EXPECT_DOUBLE_EQ(output.control.brake, 1.0);
}

TEST(SimulationEngine, PerceptionDropoutTriggersWatchdog) {
  sdc::simulation::SimulationEngine simulation(
      std::make_unique<sdc::planning::ReferencePathPlanner>());
  Mission mission;
  mission.id = "perception-dropout";
  mission.type = MissionType::kFollowRoute;
  mission.route = {{0.0, 0.0, 0.0}, {20.0, 0.0, 0.0}};
  ASSERT_TRUE(simulation.setMission(mission));
  simulation.setObstacles({});
  simulation.step(0.05);
  simulation.setFaultInjection({true, false});
  sdc::runtime::RuntimeOutput output;
  for (int i = 0; i < 12; ++i) output = simulation.step(0.05);
  EXPECT_EQ(output.state, sdc::domain::RuntimeState::kFault);
  EXPECT_DOUBLE_EQ(output.control.target_speed, 0.0);
}

TEST(SimulationEngine, EmergencyStopOverridesHealthyRuntime) {
  sdc::simulation::SimulationEngine simulation(
      std::make_unique<sdc::planning::ReferencePathPlanner>());
  Mission mission;
  mission.id = "estop";
  mission.type = MissionType::kFollowRoute;
  mission.route = {{0.0, 0.0, 0.0}, {20.0, 0.0, 0.0}};
  ASSERT_TRUE(simulation.setMission(mission));
  simulation.setObstacles({});
  simulation.requestEmergencyStop(true);
  const auto output = simulation.step(0.05);
  EXPECT_EQ(output.state, sdc::domain::RuntimeState::kEstop);
  EXPECT_TRUE(output.control.emergency_stop);
}

TEST(SimulationEngine, RecordsDeterministicReplayFrames) {
  sdc::simulation::SimulationEngine simulation(
      std::make_unique<sdc::planning::ReferencePathPlanner>());
  Mission mission;
  mission.id = "record";
  mission.type = MissionType::kFollowRoute;
  mission.route = {{0.0, 0.0, 0.0}, {20.0, 0.0, 0.0}};
  ASSERT_TRUE(simulation.setMission(mission));
  simulation.setObstacles({});
  simulation.enableRecording(true);
  simulation.step(0.05);
  simulation.step(0.05);
  ASSERT_EQ(simulation.recorder().frames().size(), 2u);
  EXPECT_LT(simulation.recorder().frames()[0].timestamp_s,
            simulation.recorder().frames()[1].timestamp_s);
  EXPECT_TRUE(simulation.recorder().frames()[0].expected_output.planning.success);
}

TEST(SimulationEngine, RunsNormalNonClosedMissionClosedLoop) {
  sdc::simulation::SimulationEngine simulation(
      std::make_unique<sdc::planning::ReferencePathPlanner>());
  simulation.reset(0.0, 0.0, 0.0);
  Mission mission;
  mission.id = "normal-route";
  mission.type = MissionType::kFollowRoute;
  mission.route = {{0.0, 0.0, 0.0}, {15.0, 0.0, 0.0}};
  mission.speed_limit = 1.5;
  ASSERT_TRUE(simulation.setMission(mission));
  simulation.setObstacles({});
  sdc::runtime::RuntimeOutput output;
  for (int i = 0; i < 20; ++i) output = simulation.step(0.05);
  EXPECT_EQ(output.state, sdc::domain::RuntimeState::kRunning);
  EXPECT_TRUE(output.planning.success);
  EXPECT_GT(simulation.vehicle().x(), 0.0);
  EXPECT_GT(simulation.vehicle().speed(), 0.0);
}
}  // namespace
