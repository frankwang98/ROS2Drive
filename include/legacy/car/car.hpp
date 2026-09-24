#ifndef SELF_DRIVING_CAR_CAR_CAR_HPP
#define SELF_DRIVING_CAR_CAR_CAR_HPP

#include "control/motor_controller.hpp"
#include "legacy/decision/decision_maker.hpp"
#include "legacy/sensor/distance_sensor.hpp"

namespace sdc {

/**
 * 小车实体：集成感知、决策、控制三大子系统，并维护行驶状态。
 */
class Car {
 public:
  Car();

  /// 推进一个时间步（dt 秒）：感知 -> 决策 -> 控制 -> 更新状态。
  void step(double dt);

  double speed() const {
    return speed_;
  }
  double distance_traveled() const {
    return distance_traveled_;
  }
  double front_distance() const {
    return front_distance_;
  }
  void set_front_distance(double d) {
    front_distance_ = d;
  }
  Action current_action() const {
    return current_action_;
  }

 private:
  DistanceSensor sensor_;
  DecisionMaker decision_maker_;
  MotorController motor_;

  double speed_{0.0};
  double distance_traveled_{0.0};
  double front_distance_{8.0};
  Action current_action_{Action::kCruise};
};

}  // namespace sdc

#endif  // SELF_DRIVING_CAR_CAR_CAR_HPP
