#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace sdc::domain {

struct Pose2D { double x{0.0}; double y{0.0}; double yaw{0.0}; };
struct Twist2D { double linear{0.0}; double angular{0.0}; };

struct VehicleState {
  Pose2D pose;
  Twist2D velocity;
  double steering_angle{0.0};
  double stamp_s{0.0};
  bool localized{false};
};

struct Obstacle {
  std::string id;
  Pose2D pose;
  double radius{0.5};
  bool dynamic{false};
};

struct Waypoint {
  Pose2D pose;
  double curvature{0.0};
  double velocity{0.0};
  double acceleration{0.0};
  double relative_time{0.0};
  bool stop_required{false};
};

struct Trajectory {
  std::string frame_id{"map"};
  double stamp_s{0.0};
  std::vector<Waypoint> points;
  bool valid{false};
};

struct ControlCommand {
  double target_speed{0.0};
  double steering_angle{0.0};
  double brake{0.0};
  bool emergency_stop{false};
};

enum class RuntimeState { kInit, kReady, kRunning, kPaused, kDegraded, kStopped, kFault, kEstop };
enum class MissionType { kNavigateTo, kFollowRoute, kStop, kPark, kDock, kReturnHome };
enum class MissionState { kPending, kActive, kPaused, kSucceeded, kFailed, kCanceled };

struct Mission {
  std::string id;
  MissionType type{MissionType::kFollowRoute};
  MissionState state{MissionState::kPending};
  std::vector<Pose2D> route;
  double speed_limit{2.0};
  double goal_tolerance{0.8};
  double timeout_s{0.0};
  double submitted_at_s{0.0};
  double started_at_s{0.0};
  double progress{0.0};
  std::string result_reason;
};

enum class FaultSeverity { kInfo, kWarning, kError, kFatal };
enum class FaultAction { kReportOnly, kDegrade, kStop, kEmergencyStop };
enum class FaultCode { kLocalizationLost, kPlanningFailed, kControlError, kVehicleError, kInputTimeout, kPerceptionTimeout, kEmergencyStop };
struct Fault { FaultCode code; FaultSeverity severity; std::string message; double stamp_s{0.0}; bool active{true}; };

}  // namespace sdc::domain
