#include "adapters/ros/runtime_message_converter.hpp"

#include <cmath>
#include <utility>

namespace sdc::ros {
namespace {
std::string runtimeName(domain::RuntimeState state) {
  static const char *names[] = {
      "INIT", "READY", "RUNNING", "PAUSED", "DEGRADED", "STOPPED", "FAULT", "ESTOP"};
  const auto value = static_cast<std::size_t>(state);
  return value < 8 ? names[value] : "UNKNOWN";
}
std::string missionStateName(domain::MissionState state) {
  static const char *names[] = {"PENDING", "ACTIVE", "PAUSED", "SUCCEEDED", "FAILED", "CANCELED"};
  const auto value = static_cast<std::size_t>(state);
  return value < 6 ? names[value] : "UNKNOWN";
}
std::string missionTypeName(domain::MissionType type) {
  static const char *names[] = {
      "NAVIGATE_TO", "FOLLOW_ROUTE", "STOP", "PARK", "DOCK", "RETURN_HOME"};
  const auto value = static_cast<std::size_t>(type);
  return value < 6 ? names[value] : "UNKNOWN";
}
std::string faultName(domain::FaultCode code) {
  static const char *names[] = {"LOCALIZATION_LOST",
                                "PLANNING_FAILED",
                                "CONTROL_ERROR",
                                "VEHICLE_ERROR",
                                "INPUT_TIMEOUT",
                                "PERCEPTION_TIMEOUT",
                                "EMERGENCY_STOP"};
  const auto value = static_cast<std::size_t>(code);
  return value < 7 ? names[value] : "UNKNOWN";
}
}  // namespace

self_driving_car_demo::msg::RuntimeStatus RuntimeMessageConverter::status(
    const runtime::VehicleRuntime &runtime, const MessageContext &c, bool localized) {
  self_driving_car_demo::msg::RuntimeStatus out;
  out.header.stamp = c.stamp;
  out.header.frame_id = c.world_frame;
  out.sequence = c.sequence;
  out.robot_id = c.robot_id;
  out.runtime_state = runtimeName(runtime.output().state);
  out.localized = localized;
  out.recovery_required = runtime.safetyRecoveryRequired();
  out.recovery_ready = runtime.safetyRecoveryReady();
  out.healthy = runtime.faults().faults().empty() && !out.recovery_required;
  const auto &mission = runtime.missions().current();
  if (mission) {
    out.mission_id = mission->id;
    out.mission_type = missionTypeName(mission->type);
    out.mission_state = missionStateName(mission->state);
    out.mission_progress = mission->progress;
    out.result_reason = mission->result_reason;
  }
  return out;
}
self_driving_car_demo::msg::Trajectory RuntimeMessageConverter::trajectory(
    const runtime::VehicleRuntime &runtime, const MessageContext &c) {
  self_driving_car_demo::msg::Trajectory out;
  out.header.stamp = c.stamp;
  out.header.frame_id = c.world_frame;
  out.sequence = c.sequence;
  const auto &mission = runtime.missions().current();
  out.mission_id = mission ? mission->id : "";
  out.valid = runtime.output().planning.trajectory.valid;
  for (const auto &p : runtime.output().planning.trajectory.points) {
    self_driving_car_demo::msg::TrajectoryPoint q;
    q.pose.position.x = p.pose.x;
    q.pose.position.y = p.pose.y;
    q.pose.position.z = p.pose.z;
    q.pose.orientation.z = std::sin(p.pose.yaw / 2.0);
    q.pose.orientation.w = std::cos(p.pose.yaw / 2.0);
    q.curvature = p.curvature;
    q.velocity = p.velocity;
    q.acceleration = p.acceleration;
    q.relative_time = p.relative_time;
    q.stop_required = p.stop_required;
    out.points.push_back(std::move(q));
  }
  return out;
}
self_driving_car_demo::msg::ControlCommand RuntimeMessageConverter::control(
    const runtime::VehicleRuntime &runtime, const MessageContext &c) {
  self_driving_car_demo::msg::ControlCommand out;
  out.header.stamp = c.stamp;
  out.header.frame_id = c.base_frame;
  out.sequence = c.sequence;
  const auto &command = runtime.output().control;
  out.target_speed = command.target_speed;
  out.steering_angle = command.steering_angle;
  out.brake = command.brake;
  out.emergency_stop = command.emergency_stop;
  return out;
}
self_driving_car_demo::msg::FaultArray RuntimeMessageConverter::faults(
    const runtime::VehicleRuntime &runtime, const MessageContext &c) {
  self_driving_car_demo::msg::FaultArray out;
  out.header.stamp = c.stamp;
  out.header.frame_id = c.base_frame;
  out.sequence = c.sequence;
  out.robot_id = c.robot_id;
  for (const auto &active : runtime.faults().faults()) {
    self_driving_car_demo::msg::Fault fault;
    fault.code = faultName(active.code);
    fault.severity = static_cast<uint8_t>(active.severity);
    fault.message = active.message;
    fault.stamp = c.stamp;
    fault.active = active.active;
    out.faults.push_back(std::move(fault));
  }
  return out;
}
self_driving_car_demo::msg::RuntimeMetrics RuntimeMessageConverter::metrics(
    const runtime::VehicleRuntime &runtime, const MessageContext &c, double duration, double rate) {
  self_driving_car_demo::msg::RuntimeMetrics out;
  out.header.stamp = c.stamp;
  out.sequence = c.sequence;
  out.robot_id = c.robot_id;
  out.loop_duration_ms = duration;
  out.configured_loop_hz = rate;
  out.trajectory_points = static_cast<uint32_t>(runtime.output().planning.trajectory.points.size());
  out.active_faults = static_cast<uint32_t>(runtime.faults().faults().size());
  out.runtime_state = static_cast<uint8_t>(runtime.output().state);
  return out;
}
}  // namespace sdc::ros
