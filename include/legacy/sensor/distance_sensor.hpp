#ifndef SELF_DRIVING_CAR_SENSOR_DISTANCE_SENSOR_HPP
#define SELF_DRIVING_CAR_SENSOR_DISTANCE_SENSOR_HPP

namespace sdc {

/**
 * 距离传感器：模拟测量小车前方障碍物的距离（单位：米）。
 */
class DistanceSensor {
 public:
  explicit DistanceSensor(double max_range = 10.0);

  /// 读取当前前方障碍物距离（米）。测量值带有轻微噪声。
  double read_distance() const;

  /// 更新传感器所处的世界状态（模拟场景中使用）。
  void update_obstacle_distance(double distance);

 private:
  double max_range_;
  double obstacle_distance_;
};

}  // namespace sdc

#endif  // SELF_DRIVING_CAR_SENSOR_DISTANCE_SENSOR_HPP
