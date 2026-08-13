#include "sim/map.hpp"

#include <algorithm>
#include <array>
#include <cmath>

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
    case MapType::kRing:          return "Ring";
    case MapType::kReverseParking: return "ReverseParking";
    case MapType::kSideParking:    return "SideParking";
    case MapType::kRightAngleTurn: return "RightAngleTurn";
  }
  return "Unknown";
}

const char* map_type_label(MapType t) {
  switch (t) {
    case MapType::kRing:          return "环形道路";
    case MapType::kReverseParking: return "倒车入库";
    case MapType::kSideParking:    return "侧方停车";
    case MapType::kRightAngleTurn: return "直角转弯";
  }
  return "?";
}

std::unique_ptr<ScenarioMap> create_map(MapType t) {
  switch (t) {
    case MapType::kRing:           return std::make_unique<RingMap>();
    case MapType::kReverseParking: return std::make_unique<ReverseParkingMap>();
    case MapType::kSideParking:    return std::make_unique<SideParkingMap>();
    case MapType::kRightAngleTurn: return std::make_unique<RightAngleTurnMap>();
  }
  return std::make_unique<RingMap>();
}

// =====================================================================
//  RingMap（环形道路，原逻辑迁移）
// =====================================================================
RingMap::RingMap(double radius, double road_width)
    : radius_(radius), road_width_(road_width) {}

void RingMap::reset(CarState& car) const {
  // 起点位于环道东侧（角度 0），朝向切线方向（π/2）
  car.reset(radius_, 0.0, M_PI_2, 0.0, 0.0);
}

Vec2 RingMap::goal_point(double /*progress*/) const {
  // 环形无终点，返回一个沿切线前方的虚拟点（自动行驶不会真正用到）
  return Vec2{radius_, radius_ * 0.1};
}

bool RingMap::goal_reached(const CarState& /*car*/) const {
  return false;  // 环道为无限行驶，不判定完成
}

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
  // 中央虚线
  {
    Marker m;
    m.header.frame_id = "world";
    m.ns = "road"; m.id = 1;
    m.type = Marker::LINE_LIST; m.action = Marker::ADD;
    m.pose.orientation.w = 1.0;
    m.scale.x = 0.15;
    m.color.r = 1.0f; m.color.g = 1.0f; m.color.b = 1.0f; m.color.a = 0.9f;
    constexpr int DASH = 100;
    for (int i = 0; i < DASH; ++i) {
      double a0 = 2.0 * M_PI * i / DASH;
      double a1 = 2.0 * M_PI * (i + 0.5) / DASH;
      m.points.push_back(make_point(radius_ * std::cos(a0), radius_ * std::sin(a0), z));
      m.points.push_back(make_point(radius_ * std::cos(a1), radius_ * std::sin(a1), z));
    }
    ma.markers.push_back(m);
  }
  // 内/外边界
  for (int k = 0; k < 2; ++k) {
    Marker m;
    m.header.frame_id = "world";
    m.ns = "road"; m.id = 2 + k;
    m.type = Marker::LINE_STRIP; m.action = Marker::ADD;
    m.pose.orientation.w = 1.0;
    m.scale.x = 0.25;
    m.color.r = 1.0f; m.color.g = 1.0f; m.color.b = 1.0f; m.color.a = 0.95f;
    double r = (k == 0) ? inner_r : outer_r;
    for (int i = 0; i <= segments_; ++i) {
      double a = 2.0 * M_PI * i / segments_;
      m.points.push_back(make_point(r * std::cos(a), r * std::sin(a), z));
    }
    ma.markers.push_back(m);
  }
  return ma;
}

std::vector<Obstacle> RingMap::to_obstacles() const {
  // 环道边界墙（内外侧各一圈圆），复用避障，防止冲出环道
  double inner_r = radius_ - road_width_ / 2.0;
  double outer_r = radius_ + road_width_ / 2.0;
  std::vector<std::array<double, 4>> segs;
  constexpr int N = 120;
  for (int k = 0; k < 2; ++k) {
    double r = (k == 0) ? inner_r : outer_r;
    for (int i = 0; i < N; ++i) {
      double a0 = 2.0 * M_PI * i / N;
      double a1 = 2.0 * M_PI * (i + 1) / N;
      segs.push_back({r * std::cos(a0), r * std::sin(a0),
                      r * std::cos(a1), r * std::sin(a1)});
    }
  }
  return wall_obstacles(segs, 0.4, 1.2);
}

// =====================================================================
//  ReverseParkingMap（倒车入库）
// =====================================================================
ReverseParkingMap::ReverseParkingMap() = default;

void ReverseParkingMap::reset(CarState& car) const {
  // 起点在库位右前方，车头朝 -y（向南），准备倒车入库
  car.reset(start_x_, start_y_, -M_PI_2, 0.0, 0.0);
}

Vec2 ReverseParkingMap::goal_point(double /*progress*/) const {
  return Vec2{bay_cx_, bay_cy_ + bay_h_ * 0.5};  // 库内侧（倒车终点）
}

bool ReverseParkingMap::goal_reached(const CarState& car) const {
  double d = std::hypot(car.x() - (bay_cx_),
                        car.y() - (bay_cy_ + bay_h_ * 0.5));
  return d < goal_tol_;
}

MarkerArray ReverseParkingMap::build_road_markers() const {
  MarkerArray ma;
  double z = 0.06;
  // 地面（库位区域浅色）
  {
    Marker m;
    m.header.frame_id = "world";
    m.ns = "road"; m.id = 0;
    m.type = Marker::TRIANGLE_LIST; m.action = Marker::ADD;
    m.pose.orientation.w = 1.0;
    m.color.r = 0.2f; m.color.g = 0.2f; m.color.b = 0.25f; m.color.a = 0.8f;
    double x0 = bay_cx_ - bay_w_ / 2, x1 = bay_cx_ + bay_w_ / 2;
    double y0 = bay_cy_ - bay_h_ / 2, y1 = bay_cy_ + bay_h_ / 2;
    m.points.push_back(make_point(x0, y0, 0.0));
    m.points.push_back(make_point(x1, y0, 0.0));
    m.points.push_back(make_point(x1, y1, 0.0));
    m.points.push_back(make_point(x0, y0, 0.0));
    m.points.push_back(make_point(x1, y1, 0.0));
    m.points.push_back(make_point(x0, y1, 0.0));
    ma.markers.push_back(m);
  }
  // 库位边框（黄线）
  {
    Marker m;
    m.header.frame_id = "world";
    m.ns = "bay"; m.id = 1;
    m.type = Marker::LINE_STRIP; m.action = Marker::ADD;
    m.pose.orientation.w = 1.0;
    m.scale.x = 0.2;
    m.color.r = 1.0f; m.color.g = 0.9f; m.color.b = 0.2f; m.color.a = 0.95f;
    double x0 = bay_cx_ - bay_w_ / 2, x1 = bay_cx_ + bay_w_ / 2;
    double y0 = bay_cy_ - bay_h_ / 2, y1 = bay_cy_ + bay_h_ / 2;
    m.points.push_back(make_point(x0, y0, z));
    m.points.push_back(make_point(x1, y0, z));
    m.points.push_back(make_point(x1, y1, z));
    m.points.push_back(make_point(x0, y1, z));
    m.points.push_back(make_point(x0, y0, z));
    ma.markers.push_back(m);
  }
  // 起点标记（绿色圆）
  {
    Marker m;
    m.header.frame_id = "world";
    m.ns = "start"; m.id = 2;
    m.type = Marker::CYLINDER; m.action = Marker::ADD;
    m.pose.position.x = start_x_; m.pose.position.y = start_y_;
    m.pose.position.z = 0.03; m.pose.orientation.w = 1.0;
    m.scale.x = 2.0; m.scale.y = 2.0; m.scale.z = 0.05;
    m.color.r = 0.2f; m.color.g = 1.0f; m.color.b = 0.3f; m.color.a = 0.6f;
    ma.markers.push_back(m);
  }
  return ma;
}

MarkerArray ReverseParkingMap::build_extra_markers() const {
  MarkerArray ma;
  Marker m;
  m.header.frame_id = "world";
  m.ns = "label"; m.id = 0;
  m.type = Marker::TEXT_VIEW_FACING; m.action = Marker::ADD;
  m.pose.position.x = bay_cx_;
  m.pose.position.y = bay_cy_ + bay_h_ / 2 + 1.0;
  m.pose.position.z = 1.5;
  m.scale.z = 1.2;
  m.color.r = 1.0f; m.color.g = 1.0f; m.color.b = 0.2f; m.color.a = 1.0f;
  m.text = "倒车入库";
  ma.markers.push_back(m);
  return ma;
}

std::vector<Obstacle> ReverseParkingMap::to_obstacles() const {
  // 库位三面墙（开口朝 +y 为入口，不挡）
  double x0 = bay_cx_ - bay_w_ / 2, x1 = bay_cx_ + bay_w_ / 2;
  double y0 = bay_cy_ - bay_h_ / 2, y1 = bay_cy_ + bay_h_ / 2;
  std::vector<std::array<double, 4>> segs = {
      {x0, y0, x1, y0},  // 后墙（-y）
      {x0, y0, x0, y1},  // 左墙
      {x1, y0, x1, y1},  // 右墙
  };
  return wall_obstacles(segs, 0.3, 0.8);
}

// =====================================================================
//  SideParkingMap（侧方停车）
// =====================================================================
SideParkingMap::SideParkingMap() = default;

void SideParkingMap::reset(CarState& car) const {
  car.reset(start_x_, start_y_, -M_PI_2, 0.0, 0.0);
}

Vec2 SideParkingMap::goal_point(double /*progress*/) const {
  return Vec2{bay_cx_, bay_cy_ + bay_h_ * 0.5};
}

bool SideParkingMap::goal_reached(const CarState& car) const {
  double d = std::hypot(car.x() - bay_cx_, car.y() - (bay_cy_ + bay_h_ * 0.5));
  return d < goal_tol_;
}

MarkerArray SideParkingMap::build_road_markers() const {
  MarkerArray ma;
  double z = 0.06;
  // 地面
  {
    Marker m;
    m.header.frame_id = "world";
    m.ns = "road"; m.id = 0;
    m.type = Marker::TRIANGLE_LIST; m.action = Marker::ADD;
    m.pose.orientation.w = 1.0;
    m.color.r = 0.2f; m.color.g = 0.2f; m.color.b = 0.25f; m.color.a = 0.8f;
    double x0 = bay_cx_ - bay_w_ / 2, x1 = bay_cx_ + bay_w_ / 2;
    double y0 = bay_cy_ - bay_h_ / 2, y1 = bay_cy_ + bay_h_ / 2;
    m.points.push_back(make_point(x0, y0, 0.0));
    m.points.push_back(make_point(x1, y0, 0.0));
    m.points.push_back(make_point(x1, y1, 0.0));
    m.points.push_back(make_point(x0, y0, 0.0));
    m.points.push_back(make_point(x1, y1, 0.0));
    m.points.push_back(make_point(x0, y1, 0.0));
    ma.markers.push_back(m);
  }
  // 库位框（前后两条边界 + 外沿）
  {
    Marker m;
    m.header.frame_id = "world";
    m.ns = "bay"; m.id = 1;
    m.type = Marker::LINE_LIST; m.action = Marker::ADD;
    m.pose.orientation.w = 1.0;
    m.scale.x = 0.2;
    m.color.r = 1.0f; m.color.g = 0.9f; m.color.b = 0.2f; m.color.a = 0.95f;
    double x0 = bay_cx_ - bay_w_ / 2, x1 = bay_cx_ + bay_w_ / 2;
    double y1 = bay_cy_ + bay_h_ / 2;
    // 前车后保险杠
    m.points.push_back(make_point(x0, bay_cy_ - bay_h_ / 2, z));
    m.points.push_back(make_point(x1, bay_cy_ - bay_h_ / 2, z));
    // 后车前保险杠
    m.points.push_back(make_point(x0, bay_cy_ + bay_h_ / 2, z));
    m.points.push_back(make_point(x1, bay_cy_ + bay_h_ / 2, z));
    // 外侧路沿
    m.points.push_back(make_point(x0, y1, z));
    m.points.push_back(make_point(x1, y1, z));
    ma.markers.push_back(m);
  }
  {
    Marker m;
    m.header.frame_id = "world";
    m.ns = "start"; m.id = 2;
    m.type = Marker::CYLINDER; m.action = Marker::ADD;
    m.pose.position.x = start_x_; m.pose.position.y = start_y_;
    m.pose.position.z = 0.03; m.pose.orientation.w = 1.0;
    m.scale.x = 2.0; m.scale.y = 2.0; m.scale.z = 0.05;
    m.color.r = 0.2f; m.color.g = 1.0f; m.color.b = 0.3f; m.color.a = 0.6f;
    ma.markers.push_back(m);
  }
  return ma;
}

MarkerArray SideParkingMap::build_extra_markers() const {
  MarkerArray ma;
  Marker m;
  m.header.frame_id = "world";
  m.ns = "label"; m.id = 0;
  m.type = Marker::TEXT_VIEW_FACING; m.action = Marker::ADD;
  m.pose.position.x = bay_cx_;
  m.pose.position.y = bay_cy_ + bay_h_ / 2 + 1.0;
  m.pose.position.z = 1.5;
  m.scale.z = 1.2;
  m.color.r = 1.0f; m.color.g = 1.0f; m.color.b = 0.2f; m.color.a = 1.0f;
  m.text = "侧方停车";
  ma.markers.push_back(m);
  return ma;
}

std::vector<Obstacle> SideParkingMap::to_obstacles() const {
  double x0 = bay_cx_ - bay_w_ / 2, x1 = bay_cx_ + bay_w_ / 2;
  double y0 = bay_cy_ - bay_h_ / 2, y1 = bay_cy_ + bay_h_ / 2;
  // 前车（库前边界）+ 后车（库后边界）+ 外侧路沿，开口朝 -y（车头方向）由车自行进入
  std::vector<std::array<double, 4>> segs = {
      {x0, y0, x1, y0},
      {x0, y1, x1, y1},
      {x0, y0, x0, y1},  // 内侧墙（贴近车道）
  };
  return wall_obstacles(segs, 0.3, 0.8);
}

// =====================================================================
//  RightAngleTurnMap（直角转弯）
// =====================================================================
RightAngleTurnMap::RightAngleTurnMap() = default;

void RightAngleTurnMap::reset(CarState& car) const {
  // 起点在横段左端，车头朝 +x
  car.reset(start_x_, start_y_, 0.0, 0.0, 0.0);
}

Vec2 RightAngleTurnMap::goal_point(double /*progress*/) const {
  return Vec2{start_x_ + len_, road_w_ + 3.0};  // 竖段上端出口
}

bool RightAngleTurnMap::goal_reached(const CarState& car) const {
  return car.y() > road_w_ + 1.0 && car.x() > 1.0;  // 已驶入竖段上方
}

MarkerArray RightAngleTurnMap::build_road_markers() const {
  MarkerArray ma;
  double z = 0.06;
  double w = road_w_;
  // 横段路面 (y ∈ [-3,0])
  {
    Marker m;
    m.header.frame_id = "world";
    m.ns = "road"; m.id = 0;
    m.type = Marker::TRIANGLE_LIST; m.action = Marker::ADD;
    m.pose.orientation.w = 1.0;
    m.color.r = 0.2f; m.color.g = 0.2f; m.color.b = 0.25f; m.color.a = 0.85f;
    double xs = start_x_ - 5.0, xe = w + 5.0;
    double ys = -3.0, ye = 0.0;
    m.points.push_back(make_point(xs, ys, 0.0));
    m.points.push_back(make_point(xe, ys, 0.0));
    m.points.push_back(make_point(xe, ye, 0.0));
    m.points.push_back(make_point(xs, ys, 0.0));
    m.points.push_back(make_point(xe, ye, 0.0));
    m.points.push_back(make_point(xs, ye, 0.0));
    ma.markers.push_back(m);
  }
  // 竖段路面 (x ∈ [0,3])
  {
    Marker m;
    m.header.frame_id = "world";
    m.ns = "road"; m.id = 1;
    m.type = Marker::TRIANGLE_LIST; m.action = Marker::ADD;
    m.pose.orientation.w = 1.0;
    m.color.r = 0.2f; m.color.g = 0.2f; m.color.b = 0.25f; m.color.a = 0.85f;
    double xs = 0.0, xe = w;
    double ys = -3.0, ye = w + len_ - 3.0;
    m.points.push_back(make_point(xs, ys, 0.0));
    m.points.push_back(make_point(xe, ys, 0.0));
    m.points.push_back(make_point(xe, ye, 0.0));
    m.points.push_back(make_point(xs, ys, 0.0));
    m.points.push_back(make_point(xe, ye, 0.0));
    m.points.push_back(make_point(xs, ye, 0.0));
    ma.markers.push_back(m);
  }
  // 边界线
  {
    Marker m;
    m.header.frame_id = "world";
    m.ns = "road"; m.id = 2;
    m.type = Marker::LINE_STRIP; m.action = Marker::ADD;
    m.pose.orientation.w = 1.0;
    m.scale.x = 0.2;
    m.color.r = 1.0f; m.color.g = 1.0f; m.color.b = 1.0f; m.color.a = 0.9f;
    // 外圈（右/下边界 + 上边界）
    m.points.push_back(make_point(start_x_ - 5.0, -3.0, z));
    m.points.push_back(make_point(w, -3.0, z));
    m.points.push_back(make_point(w, w + len_ - 3.0, z));
    ma.markers.push_back(m);
  }
  {
    Marker m;
    m.header.frame_id = "world";
    m.ns = "road"; m.id = 3;
    m.type = Marker::LINE_STRIP; m.action = Marker::ADD;
    m.pose.orientation.w = 1.0;
    m.scale.x = 0.2;
    m.color.r = 1.0f; m.color.g = 1.0f; m.color.b = 1.0f; m.color.a = 0.9f;
    // 内圈（上边界线 y=0）
    m.points.push_back(make_point(start_x_ - 5.0, 0.0, z));
    m.points.push_back(make_point(w + 5.0, 0.0, z));
    m.points.push_back(make_point(w + 5.0, w + len_ - 3.0, z));
    ma.markers.push_back(m);
  }
  return ma;
}

std::vector<Obstacle> RightAngleTurnMap::to_obstacles() const {
  double w = road_w_;
  std::vector<std::array<double, 4>> segs = {
      {start_x_ - 5.0, -3.0, w, -3.0},         // 下边界
      {w, -3.0, w, w + len_ - 3.0},            // 右边界
      {start_x_ - 5.0, 0.0, w + 5.0, 0.0},     // 内边界（y=0）横线
      {w + 5.0, 0.0, w + 5.0, w + len_ - 3.0}, // 左上边界
  };
  return wall_obstacles(segs, 0.3, 1.0);
}

}  // namespace sdc
