#include "sensor/distance_sensor.hpp"

#include <algorithm>
#include <cmath>
#include <cstdlib>

namespace sdc {

DistanceSensor::DistanceSensor(double max_range)
    : max_range_(max_range), obstacle_distance_(max_range) {}

double DistanceSensor::read_distance() const {
  // 模拟传感器噪声：±2.5% 的随机波动
  double noise = 1.0 + 0.025 * (static_cast<double>(std::rand()) / RAND_MAX - 0.5) * 2.0;
  double reading = obstacle_distance_ * noise;
  return std::clamp(reading, 0.0, max_range_);
}

void DistanceSensor::update_obstacle_distance(double distance) {
  obstacle_distance_ = std::clamp(distance, 0.0, max_range_);
}

}  // namespace sdc
