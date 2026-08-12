#ifndef SELF_DRIVING_CAR_CONTROL_STEERING_CONTROLLER_HPP
#define SELF_DRIVING_CAR_CONTROL_STEERING_CONTROLLER_HPP

#include <cmath>

namespace sdc {

/// 基于 Stanley（前轴误差 + 航向角误差）的转向控制器。
///
/// 输入小车的当前位姿、期望路径上的目标点，以及车速，输出一个平滑的前轮转角，
/// 使小车沿期望路径行驶。相比直接对横向偏移做硬赋值，该控制器通过
///   转向 = 航向角误差项 + atan(k * 前轴横向误差 / (速度 + 平滑项))
/// 将车头朝向往目标点逐渐打过去，配合阿克曼运动学，转弯自然且不会"画龙"。
class SteeringController {
 public:
  struct Params {
    double k_cte  = 1.2;   // 横向误差增益（Stanley 的 k）
    double k_head = 0.8;   // 航向角误差增益
    double min_v  = 0.5;   // 防除零的最小速度（m/s）
    double max_steer = 0.55; // 输出前轮最大转角（弧度），与运动学模型一致
  };

  explicit SteeringController();
  explicit SteeringController(const Params& p);

  /// 计算目标前轮转角。
  /// @param car_x, car_y, car_yaw 小车当前位姿
  /// @param goal_x, goal_y        期望路径上的目标点（世界坐标）
  /// @param speed                 当前车速（m/s）
  /// @return 前轮转角（弧度，正值左转）
  double compute(double car_x, double car_y, double car_yaw,
                 double goal_x, double goal_y, double speed) const;

 private:
  double clamp(double v, double lo, double hi) const { return std::fmin(hi, std::fmax(lo, v)); }

  Params p_;
};

}  // namespace sdc

#endif  // SELF_DRIVING_CAR_CONTROL_STEERING_CONTROLLER_HPP
