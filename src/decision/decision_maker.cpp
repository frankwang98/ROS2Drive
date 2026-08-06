#include "decision/decision_maker.hpp"

namespace sdc {

const char* action_name(Action action) {
  switch (action) {
    case Action::kAccelerate:
      return "加速";
    case Action::kCruise:
      return "匀速巡航";
    case Action::kBrake:
      return "减速";
    case Action::kStop:
      return "停车";
    default:
      return "未知";
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

}  // namespace sdc
