#include "control/steering_controller.hpp"

namespace sdc {

SteeringController::SteeringController() : p_(Params()) {}

SteeringController::SteeringController(const Params& p) : p_(p) {}

double SteeringController::compute(double car_x, double car_y, double car_yaw,
                                   double goal_x, double goal_y, double speed) const {
  // 1. 目标点相对小车的矢量（车体系下）。
  double dx = goal_x - car_x;
  double dy = goal_y - car_y;
  double d  = std::hypot(dx, dy);
  if (d < 1e-6) return 0.0;

  // 2. 车体前方方向与航向角误差。
  double goal_yaw = std::atan2(dy, dx);
  // 归一化到 [-pi, pi]
  double heading_err = goal_yaw - car_yaw;
  while (heading_err >  M_PI) heading_err -= 2.0 * M_PI;
  while (heading_err < -M_PI) heading_err += 2.0 * M_PI;

  // 3. 前轴横向误差：目标点在车体坐标系下的 y（左为正）。
  double c = std::cos(car_yaw), s = std::sin(car_yaw);
  double cte = -s * dx + c * dy;

  // 4. Stanley 转向律。
  double v = std::fmax(std::fabs(speed), p_.min_v);
  double steer = p_.k_head * heading_err + std::atan2(p_.k_cte * cte, v);

  return clamp(steer, -p_.max_steer, p_.max_steer);
}

}  // namespace sdc
