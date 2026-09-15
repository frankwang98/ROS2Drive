#ifndef SELF_DRIVING_CAR_SIM_MAP_HPP
#define SELF_DRIVING_CAR_SIM_MAP_HPP

#include <memory>
#include <string>
#include <vector>

#include <array>
#include <visualization_msgs/msg/marker_array.hpp>
#include <geometry_msgs/msg/point.hpp>

#include "model/ackermann_model.hpp"
#include "domain/autonomy_types.hpp"
#include "sim/scene_types.hpp"
#include "scenario/scenario_definition.hpp"

namespace sdc {

/// 小车状态（由阿克曼模型提供）。
using CarState = AckermannModel;

/// 地图类型枚举（保留环形道路，去掉科目二）。
enum class MapType {
  kRing = 0,       // 环形道路（自动 / 手动驾驶）
};

const char* map_type_name(MapType t);
const char* map_type_label(MapType t);  // 中文短名

/// 场景地图抽象基类。
///
/// 把「场景布局 / 起点终点 / 完成判定 / 障碍物」从仿真节点中抽离出来，
/// 让仿真节点只负责「感知→规划→控制→可视化」的闭环，地图只负责「画什么、
/// 车从哪出发、到哪算完成、边界墙当障碍物」。
class ScenarioMap {
 public:
  virtual ~ScenarioMap() = default;

  /// 场景名称。
  virtual std::string name() const = 0;

  /// 构建静态场景（车道线、路型、复杂路况标记），返回 Marker 列表。
  virtual visualization_msgs::msg::MarkerArray build_road_markers() const = 0;

  /// 把车重置到本场景起点，并设置初始朝向。
  virtual void reset(CarState& car) const = 0;

  /// 当前任务目标点（世界坐标）。自动行驶朝它走。
  virtual Vec2 goal_point(double progress = 0.0) const = 0;

  /// 沿行驶方向的前视目标点（世界坐标）。
  /// 环形无终点，自动行驶应持续沿环前进，故基于小车当前位置返回一个
  /// 前方虚拟点（而非固定点），否则车会停在某个固定点不再前进。
  virtual Vec2 goal_point_ahead(const CarState& car) const { return goal_point(); }

  /// 是否已完成当前任务（车抵达目标点附近）。
  virtual bool goal_reached(const CarState& car) const = 0;

  /// 边界 / 复杂路型（slalom、窄门、路障）作为障碍物（供 Lattice 避障用）。
  virtual std::vector<Obstacle> to_obstacles() const = 0;

  /// 场景向 Runtime 提供的通用参考路线。核心 planner 不知道场景几何类型。
  virtual std::vector<domain::Pose2D> reference_path() const = 0;

  /// 场景特有的可视信息（如路况标识文字），默认空。
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

/// 环形道路场景。
///
/// 在基础环形跑道基础上增加道路复杂性：
///   - 加宽道路，提供更多横向变道空间
///   - 「S 形绕桩」路段：一排交替内/外侧的桩桶，自动驾驶需蛇形穿梭
///   - 「窄门」路段：两侧收窄形成门形通道，考验居中控制
///   - 多处路面标线（车道线、人行横道、减速带）增强视觉复杂度
///   - 保留随机动态障碍物避障（由 ObstacleManager 注入）
class RingMap : public ScenarioMap {
 public:
  RingMap(double radius = 26.0, double road_width = 9.0);

  std::string name() const override { return "环形道路"; }
  visualization_msgs::msg::MarkerArray build_road_markers() const override;
  visualization_msgs::msg::MarkerArray build_extra_markers() const override;
  void reset(CarState& car) const override;
  Vec2 goal_point(double progress = 0.0) const override;
  Vec2 goal_point_ahead(const CarState& car) const override;
  bool goal_reached(const CarState& car) const override;
  std::vector<Obstacle> to_obstacles() const override;
  std::vector<domain::Pose2D> reference_path() const override;

  double radius() const { return radius_; }
  double road_width() const { return road_width_; }

 private:
  /// 计算环道上某角度处的中心线点（世界坐标）。
  Vec2 point_on_ring(double angle) const;
  /// 计算环道上某角度 + 横向偏移（米，向内为负 / 向外为正）的点。
  Vec2 point_on_ring(double angle, double lateral) const;

  double radius_;
  double road_width_;
  int segments_ = 200;
};

/// Portable ScenarioDefinition 的简易 RViz adapter。
/// 用路线带、中心线、功能区标签和静态障碍表达非环形教学场景。
class DefinitionScenarioMap final : public ScenarioMap {
 public:
  explicit DefinitionScenarioMap(scenario::ScenarioDefinition definition);
  std::string name() const override { return definition_.id; }
  visualization_msgs::msg::MarkerArray build_road_markers() const override;
  visualization_msgs::msg::MarkerArray build_extra_markers() const override;
  void reset(CarState& car) const override;
  Vec2 goal_point(double progress = 0.0) const override;
  bool goal_reached(const CarState& car) const override;
  std::vector<Obstacle> to_obstacles() const override;
  std::vector<domain::Pose2D> reference_path() const override {
    return definition_.reference_route;
  }

 private:
  scenario::ScenarioDefinition definition_;
};

/// 工厂：根据类型创建地图。
std::unique_ptr<ScenarioMap> create_map(MapType t);

}  // namespace sdc

#endif  // SELF_DRIVING_CAR_SIM_MAP_HPP
