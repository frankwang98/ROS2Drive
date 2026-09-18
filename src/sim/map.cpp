#include "sim/map.hpp"

#include "scenario/ring_scenario.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <stdexcept>
#include <utility>

namespace sdc {

using visualization_msgs::msg::Marker;
using visualization_msgs::msg::MarkerArray;

// ============ 通用工具 ============
geometry_msgs::msg::Point ScenarioMap::make_point(double x, double y, double z) {
  geometry_msgs::msg::Point p;
  p.x = x; p.y = y; p.z = z;
  return p;
}

std::vector<Obstacle> ScenarioMap::wall_obstacles(
    const std::vector<std::array<double, 4>>& segs, double radius, double step) {
  std::vector<Obstacle> out;
  for (const auto& s : segs) {
    double x0 = s[0], y0 = s[1], x1 = s[2], y1 = s[3];
    double len = std::hypot(x1 - x0, y1 - y0);
    int n = static_cast<int>(std::ceil(len / step));
    if (n < 1) n = 1;
    for (int i = 0; i <= n; ++i) {
      double t = static_cast<double>(i) / n;
      out.push_back(Obstacle{
          Vec2{x0 + (x1 - x0) * t, y0 + (y1 - y0) * t}, radius});
    }
  }
  return out;
}

// ============ 类型名 ============
const char* map_type_name(MapType t) {
  switch (t) {
    case MapType::kRing: return "Ring";
  }
  return "Unknown";
}

const char* map_type_label(MapType t) {
  switch (t) {
    case MapType::kRing: return "环形道路";
  }
  return "?";
}

std::unique_ptr<ScenarioMap> create_map(MapType t) {
  switch (t) {
    case MapType::kRing: return std::make_unique<RingMap>();
  }
  return std::make_unique<RingMap>();
}

// =====================================================================
//  RingMap（环形道路，含复杂路型）
// =====================================================================
RingMap::RingMap(double radius, double road_width)
    : radius_(radius), road_width_(road_width) {}

Vec2 RingMap::point_on_ring(double angle) const {
  return Vec2{radius_ * std::cos(angle), radius_ * std::sin(angle)};
}

Vec2 RingMap::point_on_ring(double angle, double lateral) const {
  // 法线方向（径向，向外为正）
  double nx = std::cos(angle), ny = std::sin(angle);
  double r = radius_ + lateral;
  return Vec2{r * nx, r * ny};
}

void RingMap::reset(CarState& car) const {
  // 起点位于环道东侧（角度 0），朝向切线方向（π/2，即沿环逆时针行驶）
  car.reset(radius_, 0.0, M_PI_2, 0.0, 0.0);
}

Vec2 RingMap::goal_point(double /*progress*/) const {
  // 环形无终点，返回一个沿切线前方的虚拟点（自动行驶沿环前进）
  return Vec2{radius_, radius_ * 0.12};
}

Vec2 RingMap::goal_point_ahead(const CarState& car) const {
  // 基于小车当前位置计算沿环（逆时针）前方的虚拟目标点：
  // 取小车当前环向角前方一段弧长处的中心线点，使自动驾驶持续沿环行驶，
  // 而不是停在一个固定点。
  double theta = std::atan2(car.y(), car.x());
  const double lookahead = 6.0;         // 前视弧长（米）
  double dtheta = lookahead / radius_;  // 对应角度增量
  return point_on_ring(theta + dtheta);
}

bool RingMap::goal_reached(const CarState& /*car*/) const {
  return false;  // 环道为无限行驶，不判定完成
}

std::vector<domain::Pose2D> RingMap::reference_path() const {
  return scenario::makeRingScenarioDefinition(radius_, segments_).reference_route;
}

// ---- 静态路况障碍物（slalom 桩桶 / 窄门 / 路障） ----
// 自动/手动驾驶都会把它们当作障碍物参与避障与 LIDAR。
std::vector<Obstacle> RingMap::to_obstacles() const {
  std::vector<Obstacle> obs;

  // 1. 内外环边界墙（防止冲出道路）
  double inner_r = radius_ - road_width_ / 2.0;
  double outer_r = radius_ + road_width_ / 2.0;
  std::vector<std::array<double, 4>> segs;
  constexpr int N = 160;
  for (int k = 0; k < 2; ++k) {
    double r = (k == 0) ? inner_r : outer_r;
    for (int i = 0; i < N; ++i) {
      double a0 = 2.0 * M_PI * i / N;
      double a1 = 2.0 * M_PI * (i + 1) / N;
      segs.push_back({r * std::cos(a0), r * std::sin(a0),
                      r * std::cos(a1), r * std::sin(a1)});
    }
  }
  auto walls = wall_obstacles(segs, 0.45, 1.2);
  obs.insert(obs.end(), walls.begin(), walls.end());

  // 2. S 形绕桩路段（约在角度 130°~200°，西南区域）：
  //    一排交替内/外侧的桩桶，车辆需蛇形穿梭。
  {
    const double a0 = 130.0 * M_PI / 180.0;
    const double a1 = 200.0 * M_PI / 180.0;
    constexpr int kCones = 7;
    double half = road_width_ / 2.0 - 1.2;  // 横向偏移极限（避开边界墙）
    for (int i = 0; i < kCones; ++i) {
      double t = static_cast<double>(i) / (kCones - 1);
      double ang = a0 + (a1 - a0) * t;
      // 交替内/外侧偏移（负=向内，正=向外）
      double lat = (i % 2 == 0) ? -half * 0.55 : half * 0.55;
      Vec2 p = point_on_ring(ang, lat);
      obs.push_back(Obstacle{p, 0.55});  // 桩桶
    }
  }

  // 3. 窄门路段（约在角度 20°~50°，东北区域）：
  //    两侧各一道短墙，把通道收窄到约 3m，考验居中控制。
  {
    const double a_lo = 20.0 * M_PI / 180.0;
    const double a_hi = 50.0 * M_PI / 180.0;
    const double lat_gap = 1.6;  // 门洞距中心线横向偏移（门洞宽约 2*lat_gap）
    const double wall_len = 0.28;  // 墙的角度跨度（弧度）
    constexpr int kSeg = 8;
    // 内墙（向内收窄）
    for (int i = 0; i < kSeg; ++i) {
      double a = a_lo + (a_hi - a_lo) * i / (kSeg - 1);
      Vec2 p = point_on_ring(a, -lat_gap);
      obs.push_back(Obstacle{p, 0.5});
    }
    // 外墙（向外收窄）
    for (int i = 0; i < kSeg; ++i) {
      double a = a_lo + (a_hi - a_lo) * i / (kSeg - 1);
      Vec2 p = point_on_ring(a, lat_gap);
      obs.push_back(Obstacle{p, 0.5});
    }
  }

  // 4. 静态路障（约在角度 300°，东南区域）：单个大路障，需绕行。
  {
    Vec2 p = point_on_ring(300.0 * M_PI / 180.0, 0.4);
    obs.push_back(Obstacle{p, 1.1});
  }

  return obs;
}

// ---- 道路绘制 ----
MarkerArray RingMap::build_road_markers() const {
  MarkerArray ma;
  double inner_r = radius_ - road_width_ / 2.0;
  double outer_r = radius_ + road_width_ / 2.0;
  double z = 0.06;

  // 路面
  {
    Marker m;
    m.header.frame_id = "world";
    m.ns = "road"; m.id = 0;
    m.type = Marker::TRIANGLE_LIST; m.action = Marker::ADD;
    m.pose.orientation.w = 1.0;
    m.color.r = 0.2f; m.color.g = 0.2f; m.color.b = 0.25f; m.color.a = 0.85f;
    for (int i = 0; i < segments_; ++i) {
      double a0 = 2.0 * M_PI * i / segments_;
      double a1 = 2.0 * M_PI * (i + 1) / segments_;
      m.points.push_back(make_point(inner_r * std::cos(a0), inner_r * std::sin(a0), 0.0));
      m.points.push_back(make_point(outer_r * std::cos(a0), outer_r * std::sin(a0), 0.0));
      m.points.push_back(make_point(outer_r * std::cos(a1), outer_r * std::sin(a1), 0.0));
      m.points.push_back(make_point(inner_r * std::cos(a0), inner_r * std::sin(a0), 0.0));
      m.points.push_back(make_point(outer_r * std::cos(a1), outer_r * std::sin(a1), 0.0));
      m.points.push_back(make_point(inner_r * std::cos(a1), inner_r * std::sin(a1), 0.0));
    }
    ma.markers.push_back(m);
  }

  // 中央双黄线（虚线，双线增强视觉）
  for (int line = 0; line < 2; ++line) {
    Marker m;
    m.header.frame_id = "world";
    m.ns = "road"; m.id = 1 + line;
    m.type = Marker::LINE_LIST; m.action = Marker::ADD;
    m.pose.orientation.w = 1.0;
    m.scale.x = 0.12;
    m.color.r = 1.0f; m.color.g = 0.85f; m.color.b = 0.2f; m.color.a = 0.9f;
    constexpr int DASH = 120;
    double dash_r = radius_ + (line == 0 ? -0.35 : 0.35);
    for (int i = 0; i < DASH; ++i) {
      double a0 = 2.0 * M_PI * i / DASH;
      double a1 = 2.0 * M_PI * (i + 0.5) / DASH;
      m.points.push_back(make_point(dash_r * std::cos(a0), dash_r * std::sin(a0), z));
      m.points.push_back(make_point(dash_r * std::cos(a1), dash_r * std::sin(a1), z));
    }
    ma.markers.push_back(m);
  }

  // 内/外边界（实线白边）
  for (int k = 0; k < 2; ++k) {
    Marker m;
    m.header.frame_id = "world";
    m.ns = "road"; m.id = 10 + k;
    m.type = Marker::LINE_STRIP; m.action = Marker::ADD;
    m.pose.orientation.w = 1.0;
    m.scale.x = 0.3;
    m.color.r = 1.0f; m.color.g = 1.0f; m.color.b = 1.0f; m.color.a = 0.95f;
    double r = (k == 0) ? inner_r : outer_r;
    for (int i = 0; i <= segments_; ++i) {
      double a = 2.0 * M_PI * i / segments_;
      m.points.push_back(make_point(r * std::cos(a), r * std::sin(a), z));
    }
    ma.markers.push_back(m);
  }

  // 人行横道（斑马线）—— 东南角视觉细节
  {
    Marker m;
    m.header.frame_id = "world";
    m.ns = "cross"; m.id = 0;
    m.type = Marker::TRIANGLE_LIST; m.action = Marker::ADD;
    m.pose.orientation.w = 1.0;
    m.color.r = 1.0f; m.color.g = 1.0f; m.color.b = 1.0f; m.color.a = 0.7f;
    const double a_center = 305.0 * M_PI / 180.0;
    const double span = 0.12;      // 斑马线角度跨度
    constexpr int kStripes = 6;
    for (int i = 0; i < kStripes; ++i) {
      double a = a_center + (i - kStripes / 2.0) * span;
      double a2 = a + span * 0.5;
      double in = inner_r + 0.4, out = outer_r - 0.4;
      auto p1 = Vec2{in * std::cos(a), in * std::sin(a)};
      auto p2 = Vec2{out * std::cos(a), out * std::sin(a)};
      auto p3 = Vec2{out * std::cos(a2), out * std::sin(a2)};
      auto p4 = Vec2{in * std::cos(a2), in * std::sin(a2)};
      m.points.push_back(make_point(p1.x, p1.y, 0.0));
      m.points.push_back(make_point(p2.x, p2.y, 0.0));
      m.points.push_back(make_point(p3.x, p3.y, 0.0));
      m.points.push_back(make_point(p1.x, p1.y, 0.0));
      m.points.push_back(make_point(p3.x, p3.y, 0.0));
      m.points.push_back(make_point(p4.x, p4.y, 0.0));
    }
    ma.markers.push_back(m);
  }

  return ma;
}

// ---- 复杂路况标识（文字标注） ----
MarkerArray RingMap::build_extra_markers() const {
  MarkerArray ma;
  // 减速带 / 绕桩 / 窄门 等提示文字
  const struct { double angle; double lat; const char* text; } tags[] = {
      {130.0,  road_width_ / 2.0 + 1.2, "绕桩区"},
      {20.0,   road_width_ / 2.0 + 1.2, "窄门"},
      {300.0,  road_width_ / 2.0 + 1.2, "减速带"},
  };
  for (int i = 0; i < 3; ++i) {
    Marker m;
    m.header.frame_id = "world";
    m.ns = "label"; m.id = i;
    m.type = Marker::TEXT_VIEW_FACING; m.action = Marker::ADD;
    double a = tags[i].angle * M_PI / 180.0;
    double r = radius_ + tags[i].lat;
    m.pose.position.x = r * std::cos(a);
    m.pose.position.y = r * std::sin(a);
    m.pose.position.z = 1.6;
    m.scale.z = 1.2;
    m.color.r = 1.0f; m.color.g = 0.85f; m.color.b = 0.2f; m.color.a = 1.0f;
    m.text = tags[i].text;
    ma.markers.push_back(m);
  }
  return ma;
}

DefinitionScenarioMap::DefinitionScenarioMap(
    scenario::ScenarioDefinition definition)
    : definition_(std::move(definition)) {
  std::string reason;
  if (!scenario::validate(definition_, reason))
    throw std::invalid_argument("invalid scenario definition: " + reason);
}

void DefinitionScenarioMap::reset(CarState& car) const {
  const auto& pose = definition_.initial_pose;
  car.reset(pose.x, pose.y, pose.yaw, 0.0, 0.0);
}

Vec2 DefinitionScenarioMap::goal_point(double) const {
  const auto& pose = definition_.reference_route.back();
  return {pose.x, pose.y};
}

bool DefinitionScenarioMap::goal_reached(const CarState& car) const {
  const auto goal = goal_point();
  return std::hypot(car.x() - goal.x, car.y() - goal.y) < 1.0;
}

std::vector<Obstacle> DefinitionScenarioMap::to_obstacles() const {
  std::vector<Obstacle> obstacles;
  obstacles.reserve(definition_.static_obstacles.size());
  for (const auto& obstacle : definition_.static_obstacles)
    obstacles.push_back({{obstacle.pose.x, obstacle.pose.y}, obstacle.radius});
  return obstacles;
}

MarkerArray DefinitionScenarioMap::build_road_markers() const {
  MarkerArray markers;
  Marker road;
  road.header.frame_id = "world";
  road.ns = "scenario_road";
  road.id = 0;
  road.type = Marker::LINE_STRIP;
  road.action = Marker::ADD;
  road.pose.orientation.w = 1.0;
  road.scale.x = 6.0;
  road.color.r = 0.22f;
  road.color.g = 0.22f;
  road.color.b = 0.25f;
  road.color.a = 1.0f;
  for (const auto& pose : definition_.reference_route)
    road.points.push_back(make_point(pose.x, pose.y, 0.01));
  markers.markers.push_back(road);

  Marker center = road;
  center.ns = "scenario_centerline";
  center.id = 1;
  center.scale.x = 0.15;
  center.color.r = 1.0f;
  center.color.g = 0.8f;
  center.color.b = 0.1f;
  center.color.a = 0.9f;
  center.pose.position.z = 0.04;
  markers.markers.push_back(std::move(center));
  return markers;
}

MarkerArray DefinitionScenarioMap::build_extra_markers() const {
  MarkerArray markers;
  // Mining owns explicit LOAD/DUMP/PARK markers in the ROS visualization
  // adapter.  Do not add the legacy generic first/last-route labels here.
  if (definition_.id == "mining_haul") return markers;
  const char* labels[] = {"LOAD", "DUMP"};
  const std::size_t indices[] = {0, definition_.reference_route.size() - 1};
  for (int i = 0; i < 2; ++i) {
    const auto& pose = definition_.reference_route[indices[i]];
    Marker marker;
    marker.header.frame_id = "world";
    marker.ns = "scenario_zones";
    marker.id = i;
    marker.type = Marker::TEXT_VIEW_FACING;
    marker.action = Marker::ADD;
    marker.pose.position = make_point(pose.x, pose.y, 1.8);
    marker.pose.orientation.w = 1.0;
    marker.scale.z = 1.5;
    marker.color.r = i == 0 ? 0.2f : 1.0f;
    marker.color.g = i == 0 ? 0.9f : 0.5f;
    marker.color.b = 0.2f;
    marker.color.a = 1.0f;
    marker.text = labels[i];
    markers.markers.push_back(std::move(marker));
  }
  return markers;
}

}  // namespace sdc
