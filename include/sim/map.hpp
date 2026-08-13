#ifndef SELF_DRIVING_CAR_SIM_MAP_HPP
#define SELF_DRIVING_CAR_SIM_MAP_HPP

#include <memory>
#include <string>
#include <vector>

#include <array>
#include <visualization_msgs/msg/marker_array.hpp>
#include <geometry_msgs/msg/point.hpp>

#include "model/ackermann_model.hpp"
#include "planning/lattice_planner.hpp"

namespace sdc {

/// 小车状态（由阿克曼模型提供）。
using CarState = AckermannModel;

/// 地图类型枚举（与 HUD 下拉、/sdc/set_map 话题一致）。
enum class MapType {
  kRing = 0,           // 环形道路（原有场景）
  kReverseParking,     // 倒车入库
  kSideParking,        // 侧方停车
  kRightAngleTurn,     // 直角转弯
};

const char* map_type_name(MapType t);
const char* map_type_label(MapType t);  // 中文短名（库位标签用）

/// 场景地图抽象基类。
///
/// 把"场景布局 / 起点终点 / 完成判定 / 障碍物"从仿真节点中抽离出来，
/// 让仿真节点只负责「感知→规划→控制→可视化」的闭环，地图只负责「画什么、
/// 车从哪出发、到哪算完成、边界墙当障碍物」。
///
/// 关键设计：所有场景的边界（车道线、库位框、围墙）都以「障碍物」形式
/// 暴露给 Lattice 规划器，从而**复用现有避障闭环**，小车不会被开出界外。
class ScenarioMap {
 public:
  virtual ~ScenarioMap() = default;

  /// 场景名称。
  virtual std::string name() const = 0;

  /// 构建静态场景（车道线、库位框、起点/终点标记），返回 Marker 列表。
  virtual visualization_msgs::msg::MarkerArray build_road_markers() const = 0;

  /// 把车重置到本场景起点，并设置初始朝向。
  virtual void reset(CarState& car) const = 0;

  /// 当前任务目标点（世界坐标）。考试/自动行驶都朝它走。
  /// @param progress 0..1 任务进度（用于断点续行，默认 0）
  virtual Vec2 goal_point(double progress = 0.0) const = 0;

  /// 是否已完成当前任务（车抵达目标点附近）。
  virtual bool goal_reached(const CarState& car) const = 0;

  /// 边界 / 库位边线 作为障碍物（供 Lattice 避障用）。
  virtual std::vector<Obstacle> to_obstacles() const = 0;

  /// 场景特有的可视信息（如库位编号文字），默认空。
  virtual visualization_msgs::msg::MarkerArray build_extra_markers() const {
    return visualization_msgs::msg::MarkerArray{};
  }

  // ---- 通用工具 ----
  static geometry_msgs::msg::Point make_point(double x, double y, double z = 0.0);

 protected:
  // 由一组「线段障碍」（两端点 + 半径）构造 Obstacle 列表：
  // 把线段细分为若干圆，贴近连续墙体，避障更平滑。
  static std::vector<Obstacle> wall_obstacles(
      const std::vector<std::array<double, 4>>& segs,
      double radius, double step);
};

/// 环形道路场景（原有逻辑平滑迁移）。
class RingMap : public ScenarioMap {
 public:
  RingMap(double radius = 25.0, double road_width = 6.0);

  std::string name() const override { return "环形道路"; }
  visualization_msgs::msg::MarkerArray build_road_markers() const override;
  void reset(CarState& car) const override;
  Vec2 goal_point(double progress = 0.0) const override;
  bool goal_reached(const CarState& car) const override;
  std::vector<Obstacle> to_obstacles() const override;

  double radius() const { return radius_; }
  double road_width() const { return road_width_; }

 private:
  double radius_;
  double road_width_;
  int segments_ = 200;
};

/// 倒车入库场景。
class ReverseParkingMap : public ScenarioMap {
 public:
  ReverseParkingMap();

  std::string name() const override { return "倒车入库"; }
  visualization_msgs::msg::MarkerArray build_road_markers() const override;
  visualization_msgs::msg::MarkerArray build_extra_markers() const override;
  void reset(CarState& car) const override;
  Vec2 goal_point(double progress = 0.0) const override;
  bool goal_reached(const CarState& car) const override;
  std::vector<Obstacle> to_obstacles() const override;

 private:
  // 库位（矩形）参数
  double bay_cx_ = 0.0;    // 库中心 x
  double bay_cy_ = -12.0;  // 库中心 y
  double bay_w_ = 6.0;     // 库宽（沿 x）
  double bay_h_ = 8.0;     // 库深（沿 y）
  double start_x_ = 0.0;   // 起始位（库右前方）
  double start_y_ = -2.0;
  double goal_tol_ = 1.4;  // 完成容差（米）
};

/// 侧方停车场景。
class SideParkingMap : public ScenarioMap {
 public:
  SideParkingMap();

  std::string name() const override { return "侧方停车"; }
  visualization_msgs::msg::MarkerArray build_road_markers() const override;
  visualization_msgs::msg::MarkerArray build_extra_markers() const override;
  void reset(CarState& car) const override;
  Vec2 goal_point(double progress = 0.0) const override;
  bool goal_reached(const CarState& car) const override;
  std::vector<Obstacle> to_obstacles() const override;

 private:
  double bay_cx_ = 0.0;    // 库中心 x
  double bay_cy_ = -12.0;  // 库中心 y
  double bay_w_ = 8.0;     // 库长（沿 x）
  double bay_h_ = 3.2;     // 库宽（沿 y）
  double start_x_ = 0.0;
  double start_y_ = -6.0;
  double goal_tol_ = 1.4;
};

/// 直角转弯场景。
class RightAngleTurnMap : public ScenarioMap {
 public:
  RightAngleTurnMap();

  std::string name() const override { return "直角转弯"; }
  visualization_msgs::msg::MarkerArray build_road_markers() const override;
  void reset(CarState& car) const override;
  Vec2 goal_point(double progress = 0.0) const override;
  bool goal_reached(const CarState& car) const override;
  std::vector<Obstacle> to_obstacles() const override;

 private:
  // 一条 L 形道路：横段（y∈[-3,0]）与竖段（x∈[0,3]）交汇于原点拐角
  double road_w_ = 6.0;   // 路宽
  double len_ = 30.0;     // 每段长度
  double start_x_ = -24.0;
  double start_y_ = -3.0;
  double goal_tol_ = 2.0;
};

/// 工厂：根据类型创建地图。
std::unique_ptr<ScenarioMap> create_map(MapType t);

}  // namespace sdc

#endif  // SELF_DRIVING_CAR_SIM_MAP_HPP
