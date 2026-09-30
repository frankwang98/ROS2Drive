#include "planning/ring_lane_planner.hpp"
#include "control/trajectory_controller.hpp"
#include "simulation/simulation_engine.hpp"
#include "scenario/ring_scenario.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>

namespace {
constexpr double pi = 3.14159265358979323846;
constexpr double radius = 24.5;
void require(bool condition, const std::string& message) {
  if (!condition) throw std::runtime_error(message);
}
double angle_error(double a, double b) {
  return std::atan2(std::sin(a - b), std::cos(a - b));
}
sdc::planning::PlanningInput input_at(double angle = 0.0, double d = 0.0,
                                     double heading_error = 0.0) {
  sdc::planning::PlanningInput in;
  in.vehicle.localized = true;
  in.vehicle.pose = {(radius + d) * std::cos(angle), (radius + d) * std::sin(angle),
                     angle + pi / 2.0 + heading_error};
  in.vehicle.velocity.linear = 1.5;
  in.speed_limit = 1.5;
  for (int i = 0; i <= 120; ++i) {
    double t = 2 * pi * i / 120;
    in.reference_path.push_back({radius * std::cos(t), radius * std::sin(t), t + pi / 2});
  }
  return in;
}
sdc::domain::Obstacle obstacle_at(double angle, double d = 0.0, double r = 0.8) {
  return {"blocker", {(radius + d) * std::cos(angle), (radius + d) * std::sin(angle), 0}, r};
}

void geometry_tests() {
  sdc::planning::RingLanePlanner planner;
  auto in = input_at();
  auto keep = planner.plan(in);
  require(keep.success, keep.reason);
  for (const auto& p : keep.trajectory.points) {
    require(std::abs(std::hypot(p.pose.x, p.pose.y) - radius) < 1e-10, "keep-lane radius");
    require(std::abs(p.curvature - 1.0 / radius) < 1e-10, "keep-lane curvature");
  }
  require(planner.setTargetD(3.0), "set right target");
  auto change = planner.plan(in);
  require(change.success, change.reason);
  const auto& first = change.trajectory.points.front();
  const auto& last = change.trajectory.points.back();
  require(std::hypot(first.pose.x - in.vehicle.pose.x, first.pose.y - in.vehicle.pose.y) < 1e-10,
          "trajectory starts at actual pose");
  require(std::abs(angle_error(first.pose.yaw, in.vehicle.pose.yaw)) < 1e-10, "initial heading");
  require(std::abs(std::hypot(last.pose.x, last.pose.y) - 27.5) < 1e-9, "target d");
  require(std::abs(last.curvature - 1.0 / 27.5) < 1e-10, "target curvature");
  require(std::abs(angle_error(std::atan2(last.pose.y, last.pose.x), 24.0 / radius)) < 1e-10,
          "s is measured on reference line, independent of d");
  const auto& middle = change.trajectory.points[16]; // s=4, half of L=8
  require(std::abs(std::hypot(middle.pose.x, middle.pose.y) - 26.0) < 1e-9, "quintic midpoint");
  require(std::abs(angle_error(middle.pose.yaw, 4.0 / radius + pi / 2)) > 0.1,
          "lane-change yaw includes d prime");
  // Compare analytic yaw and curvature with independent geometric differences.
  for (std::size_t i = 1; i + 1 < change.trajectory.points.size(); ++i) {
    const auto& a = change.trajectory.points[i - 1];
    const auto& b = change.trajectory.points[i];
    const auto& c = change.trajectory.points[i + 1];
    const double yaw = std::atan2(c.pose.y - a.pose.y, c.pose.x - a.pose.x);
    require(std::abs(angle_error(yaw, b.pose.yaw)) < 0.01, "geometric yaw consistency");
    const double k = angle_error(c.pose.yaw, a.pose.yaw) /
                     std::hypot(c.pose.x - a.pose.x, c.pose.y - a.pose.y);
    require(std::abs(k - b.curvature) < 0.02, "geometric curvature consistency");
    require(b.relative_time > a.relative_time, "monotonic trajectory time");
  }
  in = input_at(3.13, 1.0, -0.12);
  change = planner.plan(in);
  require(change.success, change.reason);
  require(std::abs(angle_error(change.trajectory.points.front().pose.yaw, in.vehicle.pose.yaw)) <
              1e-9, "non-tangent initial heading across seam");
  require(planner.setTargetD(0.0), "return target");
  auto back = planner.plan(input_at(3.13, 3.0));
  require(back.success, back.reason);
  require(std::abs(std::hypot(back.trajectory.points.back().pose.x,
                              back.trajectory.points.back().pose.y) - radius) < 1e-9,
          "return to left lane");
  require(!planner.setTargetD(4.0) && !planner.setTargetD(-0.5) &&
              !planner.setTargetD(std::numeric_limits<double>::quiet_NaN()), "invalid target");
  auto unchanged = planner.plan(input_at());
  require(unchanged.success, "invalid setter does not corrupt target");
  require(planner.setTargetD(1.5), "intermediate offset target");
  auto intermediate = planner.plan(input_at());
  require(intermediate.success, intermediate.reason);
  const auto& end = intermediate.trajectory.points.back().pose;
  require(std::abs(std::hypot(end.x, end.y) - 26.0) < 1e-9, "arbitrary target d");
}

void safety_tests() {
  sdc::planning::RingLanePlanner planner;
  auto in = input_at();
  in.obstacles.push_back(obstacle_at(0.85));
  auto result = planner.plan(in);
  require(result.success, "automatic lane change: " + result.reason);
  require(std::hypot(result.trajectory.points.back().pose.x,
                      result.trajectory.points.back().pose.y) > 27.4, "automatic target");
  in.obstacles = {obstacle_at(0.4, 1.5, 3.0)};
  result = planner.plan(in);
  require(!result.success && result.reason.find("ring_lane_collision_predicted") == 0,
          "both lanes blocked");
  require(!result.trajectory.valid && result.trajectory.points.empty(), "no executable collision path");
  planner.setTargetD(0.0);
  in.obstacles = {obstacle_at(0.85)};
  require(!planner.plan(in).success, "requested d never bypasses collision checks");
  in = input_at(1.0, 3.0);
  in.obstacles = {obstacle_at(0.85)};
  require(planner.plan(in).success, "passed obstacle does not block return");
  in = input_at();
  in.obstacles = {obstacle_at(-0.01, 0.0)};
  require(!planner.plan(in).success, "nearby rear obstacle overlapping start is checked");
  in = input_at(3.13);
  in.obstacles = {obstacle_at(-2.30)};
  planner.setTargetD(-1.0);
  require(planner.plan(in).success, "seam-crossing obstacle detected and avoided");
  sdc::planning::RingLanePlanner::Config cfg;
  cfg.target_d = 3.0;
  cfg.change_length = 2.0;
  sdc::planning::RingLanePlanner sharp(cfg);
  require(!sharp.plan(input_at()).success, "infeasible sharp lane change rejected");
  cfg.spacing = 0.0;
  bool threw = false;
  try { sdc::planning::RingLanePlanner invalid(cfg); } catch (const std::invalid_argument&) { threw = true; }
  require(threw, "bad configuration rejected");
  in = input_at();
  in.vehicle.localized = false;
  require(!planner.plan(in).success, "localization loss");
  in = input_at();
  in.reference_path[0].x += 3.0;
  require(!planner.plan(in).success, "non-circular reference rejected");
}

void closed_loop_test(bool manual) {
  sdc::planning::RingLanePlanner planner;
  sdc::control::PurePursuitController controller({2.0, 1.5, 1.0, 0.55});
  auto in = input_at();
  if (manual) planner.setTargetD(3.0);
  else for (double a : {0.85, 3.0, 5.15}) in.obstacles.push_back(obstacle_at(a));
  double progress = 0.0, previous_angle = 0.0, min_clearance = 1e9;
  double max_d = 0.0;
  bool returned_left = false;
  const double dt = 0.05;
  for (int i = 0; i < 3600; ++i) {
    if (manual && i == 900) require(planner.setTargetD(0.0), "live target switch");
    in.now_s = i * dt;
    auto result = planner.plan(in);
    require(result.success, "closed loop step " + std::to_string(i) + ": " + result.reason);
    auto command = controller.compute({in.vehicle, result.trajectory, dt});
    const double speed = 1.5;
    in.vehicle.pose.x += speed * std::cos(in.vehicle.pose.yaw) * dt;
    in.vehicle.pose.y += speed * std::sin(in.vehicle.pose.yaw) * dt;
    in.vehicle.pose.yaw += speed / 2.0 * std::tan(command.steering_angle) * dt;
    const double theta = std::atan2(in.vehicle.pose.y, in.vehicle.pose.x);
    progress += angle_error(theta, previous_angle);
    previous_angle = theta;
    const double d = std::hypot(in.vehicle.pose.x, in.vehicle.pose.y) - radius;
    max_d = std::max(max_d, d);
    require(d > -0.9 && d < 3.9, "vehicle remains in road corridor");
    if (d < 0.4 && max_d > 2.5) returned_left = true;
    for (const auto& obstacle : in.obstacles)
      min_clearance = std::min(min_clearance, std::hypot(in.vehicle.pose.x - obstacle.pose.x,
                                                       in.vehicle.pose.y - obstacle.pose.y));
  }
  require(progress > 2 * pi, "closed loop completes a lap");
  require(max_d > 2.5 && returned_left, "closed loop changes lane and returns");
  require(manual || min_clearance > 1.4, "vehicle clears inflated obstacles");
  std::cout << (manual ? "requested d" : "automatic obstacles") << ": "
            << progress / (2 * pi) << " laps, max d=" << max_d
            << ", min clearance=" << min_clearance << '\n';
}

void runtime_integration_test() {
  sdc::simulation::SimulationEngine engine(std::make_unique<sdc::planning::RingLanePlanner>());
  engine.setController(std::make_unique<sdc::control::PurePursuitController>(
      sdc::control::PurePursuitController::Config{2.0, 1.5, 1.0, 0.55}));
  auto scenario = sdc::scenario::makeRingScenarioDefinition();
  std::string reason;
  auto mission = scenario.materializeDefaultMission(reason);
  require(reason.empty(), "ring mission materializes");
  engine.reset(scenario.initial_pose.x, scenario.initial_pose.y, scenario.initial_pose.yaw);
  require(engine.setMission(mission), "ring mission accepted");
  engine.setObstacles({obstacle_at(0.85), obstacle_at(3.0), obstacle_at(5.15)});
  double progress = 0.0, previous_angle = 0.0, max_d = 0.0;
  bool returned_left = false;
  for (int i = 0; i < 4000; ++i) {
    auto out = engine.step(0.05);
    require(out.planning.success, "runtime planning step " + std::to_string(i) + ": " +
                                     out.planning.reason);
    require(out.state == sdc::domain::RuntimeState::kRunning, "runtime remains running");
    const auto& car = engine.vehicle();
    const double theta = std::atan2(car.y(), car.x());
    progress += angle_error(theta, previous_angle);
    previous_angle = theta;
    const double d = std::hypot(car.x(), car.y()) - radius;
    max_d = std::max(max_d, d);
    returned_left = returned_left || (max_d > 2.5 && d < 0.4);
    require(d > -0.9 && d < 3.9, "runtime vehicle stays in corridor");
  }
  require(progress > 2 * pi && returned_left, "runtime completes lap and lane changes");
  std::cout << "full runtime: " << progress / (2 * pi) << " laps, max d=" << max_d << '\n';
}
}  // namespace

int main() {
  try {
    geometry_tests();
    safety_tests();
    closed_loop_test(false);
    closed_loop_test(true);
    runtime_integration_test();
    std::cout << "PASS: ring Frenet geometry, safety and closed-loop tracking\n";
  } catch (const std::exception& e) {
    std::cerr << "FAIL: " << e.what() << '\n';
    return 1;
  }
}
