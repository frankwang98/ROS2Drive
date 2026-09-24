#include "legacy/car/car.hpp"

namespace sdc {

Car::Car() = default;

void Car::step(double dt) {
  // 感知：更新传感器世界状态并读取测量值
  sensor_.update_obstacle_distance(front_distance_);
  double measured = sensor_.read_distance();

  // 决策
  current_action_ = decision_maker_.decide(measured);

  // 控制
  speed_ = motor_.update(current_action_, speed_, dt);

  // 更新状态
  double delta = speed_ * dt;
  distance_traveled_ += delta;
  front_distance_ -= delta;
}

}  // namespace sdc
