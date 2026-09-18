#include "decision/decision_maker.hpp"

namespace sdc {

const char* action_name(Action action) {
  switch (action) {
    case Action::kAccelerate:
      return "ACCELERATE";
    case Action::kCruise:
      return "CRUISE";
    case Action::kBrake:
      return "BRAKE";
    case Action::kStop:
      return "STOP";
    default:
      return "UNKNOWN";
  }
}

DecisionMaker::DecisionMaker(double safe_stop_distance)
    : safe_stop_distance_(safe_stop_distance) {}

Action DecisionMaker::decide(double distance) const {
  if (distance > 8.0) return Action::kAccelerate;
  if (distance > 4.0) return Action::kCruise;
  if (distance > safe_stop_distance_) return Action::kBrake;
  return Action::kStop;
}

Action DecisionMaker::decide_by_speed(double speed) const {
  // 与 decide(distance) 的档位保持一致：
  //   速度 >= 2.0 视作畅行（对应距离 > 8m） -> 加速
  //   速度 >= 1.0 视作巡航                   -> 匀速
  //   速度 >  0   视作靠近障碍                -> 减速
  //   否则停车（速度 <= 0）
  if (speed >= 2.0) return Action::kAccelerate;
  if (speed >= 1.0) return Action::kCruise;
  if (speed > 0.0)  return Action::kBrake;
  return Action::kStop;
}

}  // namespace sdc
