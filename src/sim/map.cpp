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
    case MapType::kRing:       return "Ring";
    case MapType::kExamTrack:  return "ExamTrack";
  }
  return "Unknown";
}

const char* map_type_label(MapType t) {
  switch (t) {
    case MapType::kRing:       return "环形道路";
    case MapType::kExamTrack:  return "科目二综合赛道";
  }
  return "?";
}

std::unique_ptr<ScenarioMap> create_map(MapType t) {
  switch (t) {
    case MapType::kRing:       return std::make_unique<RingMap>();
    case MapType::kExamTrack:  return std::make_unique<ExamTrackMap>();
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
//  ExamTrackMap（科目二综合赛道：一条道路上布置 3 个科目）
// =====================================================================
ExamTrackMap::ExamTrackMap() {
  // 主路：横向道路 y∈[-4,4]，x∈[-26,24]，车从左侧进入向东行驶。
  // 每个站点由一组航点描述；到达该组最后一个航点后 advance() 会
  // 返回 true，表示该科目完成。
  //
  // 站点 0：倒车入库 —— 主路南侧库位
  //   航点0：主路上库位正北的进入点（车从主路向南驶入库位）
  //   航点1：库位内侧（倒车终点）
  //   航点2：回到主路（出库）
  //
  // 站点 1：侧方停车 —— 主路南侧长条库位
  //   航点3：主路上库位进入点
  //   航点4：库位内
  //   航点5：回到主路
  //
  // 站点 2：直角转弯 —— 主路东端 L 形弯道
  //   航点6：弯道前主路
  //   航点7：L 形弯道出口（向上）
  waypoints_ = {
      // 站点 0 倒车入库
      {Vec2{-14.0, -2.0}, false},   // wp0 主路进入点（车朝南）
      {Vec2{-14.0, -8.5}, false},   // wp1 库内（倒车入库）
      {Vec2{-14.0, -2.0}, true},    // wp2 出库回到主路（倒车）
      // 站点 1 侧方停车
      {Vec2{-2.0, -2.0}, false},    // wp3 主路进入点
      {Vec2{-2.0, -8.5}, false},    // wp4 侧方库内
      {Vec2{-2.0, -2.0}, true},     // wp5 回到主路（倒车）
      // 站点 2 直角转弯
      {Vec2{12.0, -2.0}, false},    // wp6 弯道前主路
      {Vec2{16.0, 11.0}, false},    // wp7 L 形弯道出口
  };
}

void ExamTrackMap::reset(CarState& car) const {
  // 赛道起点：主路左端，车头朝东（+x）
  car.reset(ROAD_LEFT_ + 1.0, 0.0, 0.0, 0.0, 0.0);
}

Vec2 ExamTrackMap::goal_point(double /*progress*/) const {
  if (current_wp_ < 0 || current_wp_ >= static_cast<int>(waypoints_.size()))
    return Vec2{ROAD_RIGHT_, 0.0};
  return waypoints_[current_wp_].pos;
}

bool ExamTrackMap::goal_reached(const CarState& car) const {
  if (current_wp_ < 0 || current_wp_ >= static_cast<int>(waypoints_.size()))
    return false;
  const Vec2& g = waypoints_[current_wp_].pos;
  // 最后一段（直角转弯出口）用更宽判据
  double tol = (current_wp_ == 7) ? 2.5 : 1.3;
  return std::hypot(car.x() - g.x, car.y() - g.y) < tol;
}

bool ExamTrackMap::current_allow_reverse() const {
  if (current_wp_ < 0 || current_wp_ >= static_cast<int>(waypoints_.size()))
    return false;
  return waypoints_[current_wp_].allow_reverse;
}

int ExamTrackMap::current_station() const {
  for (int s = 0; s < 3; ++s) {
    if (current_wp_ >= station_start_[s] && current_wp_ < station_start_[s + 1])
      return s;
  }
  return 0;
}

bool ExamTrackMap::advance() {
  // 已是最后一个航点：到达即完成最后一个科目（直角转弯）
  if (current_wp_ >= static_cast<int>(waypoints_.size()) - 1)
    return true;
  ++current_wp_;
  // 判断是否跨越站点边界
  for (int s = 1; s <= 3; ++s) {
    if (current_wp_ == station_start_[s])
      return true;  // 刚进入新站点，说明上一个站点完成
  }
  return false;
}

void ExamTrackMap::reset_to_station(int station) {
  if (station < 0) station = 0;
  if (station > 2) station = 2;
  current_wp_ = station_start_[station];
}

const char* ExamTrackMap::station_name(int station) {
  switch (station) {
    case 0: return "倒车入库";
    case 1: return "侧方停车";
    case 2: return "直角转弯";
  }
  return "?";
}

// 画一条横向主路路面（y 从 bottom 到 top）
static void add_road_quad(MarkerArray& ma, const std::string& ns, int id,
                          double x0, double x1, double y0, double y1) {
  Marker m;
  m.header.frame_id = "world";
  m.ns = ns; m.id = id;
  m.type = Marker::TRIANGLE_LIST; m.action = Marker::ADD;
  m.pose.orientation.w = 1.0;
  m.color.r = 0.2f; m.color.g = 0.2f; m.color.b = 0.25f; m.color.a = 0.82f;
  m.points.push_back(ScenarioMap::make_point(x0, y0, 0.0));
  m.points.push_back(ScenarioMap::make_point(x1, y0, 0.0));
  m.points.push_back(ScenarioMap::make_point(x1, y1, 0.0));
  m.points.push_back(ScenarioMap::make_point(x0, y0, 0.0));
  m.points.push_back(ScenarioMap::make_point(x1, y1, 0.0));
  m.points.push_back(ScenarioMap::make_point(x0, y1, 0.0));
  ma.markers.push_back(m);
}

// 画一个矩形框（库位/区域），color 控制
static void add_rect_frame(MarkerArray& ma, const std::string& ns, int id,
                           double cx, double cy, double w, double h,
                           float r, float g, float b) {
  double x0 = cx - w / 2, x1 = cx + w / 2;
  double y0 = cy - h / 2, y1 = cy + h / 2;
  Marker m;
  m.header.frame_id = "world";
  m.ns = ns; m.id = id;
  m.type = Marker::LINE_STRIP; m.action = Marker::ADD;
  m.pose.orientation.w = 1.0;
  m.scale.x = 0.2;
  m.color.r = r; m.color.g = g; m.color.b = b; m.color.a = 0.95f;
  double z = 0.06;
  m.points.push_back(ScenarioMap::make_point(x0, y0, z));
  m.points.push_back(ScenarioMap::make_point(x1, y0, z));
  m.points.push_back(ScenarioMap::make_point(x1, y1, z));
  m.points.push_back(ScenarioMap::make_point(x0, y1, z));
  m.points.push_back(ScenarioMap::make_point(x0, y0, z));
  ma.markers.push_back(m);
}

// 画一条直线段
static void add_line(MarkerArray& ma, const std::string& ns, int id,
                     double x0, double y0, double x1, double y1,
                     float r, float g, float b, double scale = 0.18) {
  Marker m;
  m.header.frame_id = "world";
  m.ns = ns; m.id = id;
  m.type = Marker::LINE_STRIP; m.action = Marker::ADD;
  m.pose.orientation.w = 1.0;
  m.scale.x = scale;
  m.color.r = r; m.color.g = g; m.color.b = b; m.color.a = 0.9f;
  double z = 0.06;
  m.points.push_back(ScenarioMap::make_point(x0, y0, z));
  m.points.push_back(ScenarioMap::make_point(x1, y1, z));
  ma.markers.push_back(m);
}

MarkerArray ExamTrackMap::build_road_markers() const {
  MarkerArray ma;
  constexpr double TOP = 4.0, BOT = -4.0;
  constexpr double LEFT = -26.0, RIGHT = 24.0;

  // ---- 主路路面（横向道路） ----
  add_road_quad(ma, "road", 0, LEFT, RIGHT, BOT, TOP);

  // ---- 主路边界白线 ----
  add_line(ma, "road", 1, LEFT, BOT, RIGHT, BOT, 1, 1, 1);
  add_line(ma, "road", 2, LEFT, TOP, RIGHT, TOP, 1, 1, 1);
  // 中央虚线
  {
    Marker m;
    m.header.frame_id = "world";
    m.ns = "road"; m.id = 3;
    m.type = Marker::LINE_LIST; m.action = Marker::ADD;
    m.pose.orientation.w = 1.0;
    m.scale.x = 0.12;
    m.color.r = 1.0f; m.color.g = 1.0f; m.color.b = 1.0f; m.color.a = 0.8f;
    double z = 0.06;
    constexpr double DASH = 3.0;
    for (double x = LEFT; x < RIGHT; x += DASH) {
      m.points.push_back(make_point(x, 0.0, z));
      m.points.push_back(make_point(std::min(x + DASH / 2, RIGHT), 0.0, z));
    }
    ma.markers.push_back(m);
  }

  // ---- 站点 1：倒车入库（库位南侧，y≈-11） ----
  // 库位开口朝北（对准主路），从主路向南驶入
  {
    // 库位地面
    add_road_quad(ma, "bay", 10, -17.0, -11.0, -15.0, -7.0);
    // 库位黄线（三面 + 开口朝北）
    add_rect_frame(ma, "bay", 11, -14.0, -11.0, 6.0, 8.0, 1.0f, 0.9f, 0.2f);
  }

  // ---- 站点 2：侧方停车（主路南侧长条库位） ----
  {
    add_road_quad(ma, "bay", 12, -6.0, 2.0, -11.0, -7.8);
    add_rect_frame(ma, "bay", 13, -2.0, -9.4, 8.0, 3.2, 1.0f, 0.9f, 0.2f);
  }

  // ---- 站点 3：直角转弯（主路东端 L 形弯道） ----
  {
    // 竖向路面（北向出口）
    add_road_quad(ma, "road", 14, 13.0, 19.0, TOP, 15.0);
    // 内/外边界
    add_line(ma, "road", 15, 13.0, TOP, 13.0, 15.0, 1, 1, 1);   // 内线
    add_line(ma, "road", 16, 19.0, TOP, 19.0, 15.0, 1, 1, 1);   // 外线
    add_line(ma, "road", 17, 13.0, 15.0, 19.0, 15.0, 1, 1, 1);  // 顶端线
  }

  return ma;
}

MarkerArray ExamTrackMap::build_extra_markers() const {
  MarkerArray ma;
  // 各站点名称文字
  const char* names[] = {"倒车入库", "侧方停车", "直角转弯"};
  double px[] = {-14.0, -2.0, 16.0};
  double py[] = {-16.0, -12.5, 16.5};
  for (int i = 0; i < 3; ++i) {
    Marker m;
    m.header.frame_id = "world";
    m.ns = "label"; m.id = i;
    m.type = Marker::TEXT_VIEW_FACING; m.action = Marker::ADD;
    m.pose.position.x = px[i]; m.pose.position.y = py[i]; m.pose.position.z = 1.5;
    m.scale.z = 1.2;
    m.color.r = 1.0f; m.color.g = 1.0f; m.color.b = 0.2f; m.color.a = 1.0f;
    m.text = names[i];
    ma.markers.push_back(m);
  }
  return ma;
}

}  // namespace sdc
