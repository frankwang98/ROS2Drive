#ifndef SELF_DRIVING_CAR_DECISION_DECISION_MAKER_HPP
#define SELF_DRIVING_CAR_DECISION_DECISION_MAKER_HPP

namespace sdc {

/// 小车可执行的行为。
enum class Action {
  kAccelerate,  // 加速
  kCruise,      // 匀速巡航
  kBrake,       // 减速
  kStop         // 停车
};

const char* action_name(Action action);

/**
 * 决策器：根据前方障碍物距离，输出当前应采取的行为。
 * 简单规则：
 *   - 距离 > 8.0 m        -> 加速
 *   - 距离 > 4.0 m        -> 匀速巡航
 *   - 距离 > 1.5 m        -> 减速
 *   - 距离 <= 1.5 m       -> 停车
 */
class DecisionMaker {
 public:
  explicit DecisionMaker(double safe_stop_distance = 1.5);

  Action decide(double distance) const;

 private:
  double safe_stop_distance_;
};

}  // namespace sdc

#endif  // SELF_DRIVING_CAR_DECISION_DECISION_MAKER_HPP
