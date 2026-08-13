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
///
/// 仅 2 张地图：
///   - kRing      环形道路：原有功能，HUD 提供「开始 / 暂停」，保留随机障碍避障。
///   - kExamTrack 科目二综合赛道：倒车入库 / 侧方停车 / 直角转弯 三个科目
///                都布置在同一条道路上，HUD 提供「开始考试」，**无障碍物**。
enum class MapType {
  kRing = 0,       // 环形道路（开始 / 暂停）
  kExamTrack = 1,  // 科目二综合赛道（开始考试）
};

const char* map_type_name(MapType t);
const char* map_type_label(MapType t);  // 中文短名（下拉 / 库位标签用）

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

  /// 构建静态场景（车道线、库位框、起点/终点标记），返回 Marker 列表。
  virtual visualization_msgs::msg::MarkerArray build_road_markers() const = 0;

  /// 把车重置到本场景起点，并设置初始朝向。
  virtual void reset(CarState& car) const = 0;

  /// 当前任务目标点（世界坐标）。考试/自动行驶都朝它走。
  virtual Vec2 goal_point(double progress = 0.0) const = 0;

  /// 是否已完成当前任务（车抵达目标点附近）。
  virtual bool goal_reached(const CarState& car) const = 0;

  /// 边界 / 库位边线 作为障碍物（供 Lattice 避障用）。
  virtual std::vector<Obstacle> to_obstacles() const = 0;

  /// 场景特有的可视信息（如库位编号文字），默认空。
  virtual visualization_msgs::msg::MarkerArray build_extra_markers() const {
    return visualization_msgs::msg::MarkerArray{};
  }

  // ---- 航点控制（科目二赛道用，其它地图返回默认值） ----
  /// 当前航点是否允许倒车（仅科目二赛道有意义）。
  virtual bool current_allow_reverse() const { return false; }
  /// 推进到下一航点；跨越站点边界返回 true（表示一个科目完成）。
  virtual bool advance() { return false; }

  // ---- 通用工具 ----
  static geometry_msgs::msg::Point make_point(double x, double y, double z = 0.0);

 protected:
  // 由一组「线段障碍」（两端点 + 半径）构造 Obstacle 列表：
  // 把线段细分为若干圆，贴近连续墙体，避障更平滑。
  static std::vector<Obstacle> wall_obstacles(
      const std::vector<std::array<double, 4>>& segs,
      double radius, double step);
};

/// 环形道路场景（原有逻辑迁移，保留随机障碍避障 + 开始/暂停）。
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

/// 科目二综合赛道场景。
///
/// 「一条道路」上依次布置 3 个科目站点：
///   1. 倒车入库  2. 侧方停车  3. 直角转弯
/// 赛道是一条横向主路（y∈[-4,4]），车从左侧进入，沿道路向东行驶；
/// 每个站点在路边设有库位/弯道，车驶入完成该科目后回到主路继续前进。
///
/// 无障碍物：本场景 to_obstacles() 返回空，车只沿目标点循迹行驶，
/// 不参与随机障碍避障。
///
/// 行驶通过一系列「航点」推进：车依次到达各航点，到达当前航点后
/// 由外部调用 advance() 进入下一个航点；跨越站点边界时置站点完成标记，
/// 供 ExamManager 推进到下一个科目。
class ExamTrackMap : public ScenarioMap {
 public:
  /// 单个行驶航点。
  struct Waypoint {
    Vec2 pos;              // 世界坐标目标点
    bool allow_reverse;    // 是否允许倒车（倒车入库 / 出库时用）
  };

  ExamTrackMap();

  std::string name() const override { return "科目二综合赛道"; }

  visualization_msgs::msg::MarkerArray build_road_markers() const override;
  visualization_msgs::msg::MarkerArray build_extra_markers() const override;
  void reset(CarState& car) const override;
  Vec2 goal_point(double progress = 0.0) const override;
  bool goal_reached(const CarState& car) const override;
  std::vector<Obstacle> to_obstacles() const override { return {}; }  // 无障碍物

  // ---- 站点 / 航点控制（供仿真节点驱动考试流程） ----
  /// 当前航点索引。
  int current_waypoint() const { return current_wp_; }
  /// 当前航点是否允许倒车。
  bool current_allow_reverse() const;
  /// 当前航点所属站点（0=倒车入库 1=侧方停车 2=直角转弯）。
  int current_station() const;
  /// 站点总数（3）。
  int station_count() const { return 3; }
  /// 推进到下一个航点；若跨越站点边界则返回 true（表示一个科目完成）。
  /// @return 是否刚完成一个站点（需要 ExamManager 推进到下一个科目）
  bool advance();
  /// 把航点/车重置到某站点的起点（exam 开始时或切到下一科目时调用）。
  void reset_to_station(int station);
  /// 获取某站点名称。
  static const char* station_name(int station);

 private:
  // 站点边界（航点下标区间）。站点 i 的航点范围是 [station_start_[i], station_start_[i+1])
  std::array<int, 4> station_start_{0, 3, 6, 8};

  std::vector<Waypoint> waypoints_;
  int current_wp_{0};

  // 主路几何参数
  static constexpr double ROAD_TOP_ = 4.0;    // 主路上边界 y
  static constexpr double ROAD_BOTTOM_ = -4.0; // 主路下边界 y
  static constexpr double ROAD_LEFT_ = -26.0;  // 主路左端 x
  static constexpr double ROAD_RIGHT_ = 24.0;  // 主路右端 x
};

/// 工厂：根据类型创建地图。
std::unique_ptr<ScenarioMap> create_map(MapType t);

}  // namespace sdc

#endif  // SELF_DRIVING_CAR_SIM_MAP_HPP
