/**
 * ring_road_sim_node.cpp — 可切换场景的自动驾驶仿真节点
 *
 * 在原「环形道路仿真」基础上做了场景抽象重构：
 *   - 场景（地图）由 ScenarioMap 抽象（环形 / 倒车入库 / 侧方停车 / 直角转弯）
 *   - 自动驾驶闭环（感知→Lattice 避障→决策→速度→Stanley→阿克曼）封装到 AutoDriver
 *   - 科目二考试由 ExamManager 编排（开始考试→逐项完成→下一个→合格）
 *
 * 功能：
 *   1. RViz2 Marker 绘制当前场景
 *   2. Lattice 局部规划避障并可视化候选/最优路径（复用现有闭环）
 *   3. PID / Bang-Bang / Ramp 速度控制算法（话题可切换）
 *   4. 感知 → 决策 → 控制 闭环，小车自动循迹行驶
 *   5. 科目二考试：HUD「开始考试」后车自动依次完成各科目
 *   6. TF / LIDAR / 状态文本等可视化
 *
 * 新增话题：
 *   订阅：
 *     /sdc/set_map      (Int32)  切换地图 0=环形 1=倒车入库 2=侧方停车 3=直角转弯
 *     /sdc/start_exam   (Bool)   开始科目二考试（true 开始）
 *     /sdc/reset_car    (Bool)   把车放回当前地图起点
 *     /sdc/pause        (Bool)   暂停/继续
 *     /sdc/clear_trail  (Bool)   清除轨迹
 *     /sdc/control_algo (Int32)  切换速度控制算法
 *   发布：
 *     /sdc/exam_status  (String) 考试状态文本
 *     /sdc/exam_item    (Int32)  当前科目序号（-1=无）
 *     /sdc/exam_progress(String)  进度文本 如 "1/3 倒车入库"
 *     （其余 HUD 话题保持原样：speed / action_id / front_distance / obstacle_count）
 */

#include <chrono>
#include <cmath>
#include <cstdio>
#include <deque>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include "control/velocity_controller.hpp"
#include "planning/lattice_planner.hpp"
#include "sim/driver.hpp"
#include "sim/exam.hpp"
#include "sim/map.hpp"

using namespace std::chrono_literals;

// ========== 仿真参数 ==========
constexpr double SIM_DT          = 0.05;   // 仿真步长（秒）
constexpr double LIDAR_RANGE     = 35.0;   // LIDAR 探测距离（米）
constexpr int    LIDAR_BEAMS     = 360;    // 水平扫描线数
constexpr int    PATH_SAMPLES    = 60;     // 规划路径采样点
constexpr double PATH_LENGTH     = 30.0;   // 规划路径可视长度（米）
constexpr int    TRAIL_MAX_POINTS = 600;   // 行驶轨迹最多保留点数
constexpr double CAR_HEAD_Z      = 2.6;    // 状态文本高度

// ========== 工具函数 ==========
static geometry_msgs::msg::Point make_point(double x, double y, double z = 0.0) {
  geometry_msgs::msg::Point p;
  p.x = x; p.y = y; p.z = z;
  return p;
}

static MapType map_type_from_int(int v) {
  switch (v) {
    case 1: return MapType::kReverseParking;
    case 2: return MapType::kSideParking;
    case 3: return MapType::kRightAngleTurn;
    default: return MapType::kRing;
  }
}

class RingRoadSimNode : public rclcpp::Node {
public:
  RingRoadSimNode()
      : Node("ring_road_sim")
      , map_(create_map(MapType::kRing))
      , driver_()
      , sim_time_(0.0)
      , obstacle_manager_(25.0, 6.0, 20240812)
  {
    auto road_qos = rclcpp::QoS(10).transient_local();
    auto live_qos = rclcpp::QoS(10).reliable();
    road_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
        "simulation/markers", road_qos);
    live_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
        "simulation/markers_live", live_qos);
    obstacle_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
        "simulation/obstacles", live_qos);
    plan_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
        "simulation/lattice", live_qos);
    lidar_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
        "sensor/lidar", live_qos);

    // HUD 面板状态话题
    speed_pub_    = create_publisher<std_msgs::msg::Float64>("sdc/speed", live_qos);
    action_pub_   = create_publisher<std_msgs::msg::Float64>("sdc/action_id", live_qos);
    distance_pub_ = create_publisher<std_msgs::msg::Float64>("sdc/front_distance", live_qos);
    obstacle_pub2_ = create_publisher<std_msgs::msg::Float64>("sdc/obstacle_count", live_qos);

    // 考试状态话题（新增）
    exam_status_pub_  = create_publisher<std_msgs::msg::String>("sdc/exam_status", live_qos);
    exam_item_pub_    = create_publisher<std_msgs::msg::Int32>("sdc/exam_item", live_qos);
    exam_progress_pub_= create_publisher<std_msgs::msg::String>("sdc/exam_progress", live_qos);

    // 暂停/继续
    pause_sub_ = create_subscription<std_msgs::msg::Bool>(
        "sdc/pause", 10, [this](const std_msgs::msg::Bool::SharedPtr msg) {
          paused_ = msg->data;
          RCLCPP_INFO(get_logger(), paused_ ? "仿真已暂停" : "仿真已继续");
        });
    // 清除轨迹
    clear_sub_ = create_subscription<std_msgs::msg::Bool>(
        "sdc/clear_trail", 10, [this](const std_msgs::msg::Bool::SharedPtr msg) {
          if (msg->data) { trail_.clear(); RCLCPP_INFO(get_logger(), "行驶轨迹已清除"); }
        });
    // 切换速度控制算法
    algo_sub_ = create_subscription<std_msgs::msg::Int32>(
        "sdc/control_algo", 10, [this](const std_msgs::msg::Int32::SharedPtr msg) {
          int a = msg->data;
          if (a == 1)      driver_.velocity_ctrl().set_algorithm(VelocityAlgorithm::kBangBang);
          else if (a == 2) driver_.velocity_ctrl().set_algorithm(VelocityAlgorithm::kRamp);
          else             driver_.velocity_ctrl().set_algorithm(VelocityAlgorithm::kPid);
          RCLCPP_INFO(get_logger(), "控制算法切换为: %s",
                      velocity_algorithm_name(driver_.velocity_ctrl().algorithm()));
        });
    // 切换地图
    set_map_sub_ = create_subscription<std_msgs::msg::Int32>(
        "sdc/set_map", 10, [this](const std_msgs::msg::Int32::SharedPtr msg) {
          switch_map(map_type_from_int(msg->data));
        });
    // 开始考试
    start_exam_sub_ = create_subscription<std_msgs::msg::Bool>(
        "sdc/start_exam", 10, [this](const std_msgs::msg::Bool::SharedPtr msg) {
          if (msg->data) {
            exam_.start();
            RCLCPP_INFO(get_logger(), "科目二考试开始");
          }
        });
    // 重置小车到起点
    reset_sub_ = create_subscription<std_msgs::msg::Bool>(
        "sdc/reset_car", 10, [this](const std_msgs::msg::Bool::SharedPtr msg) {
          if (msg->data) { driver_.reset(map_.get()); trail_.clear();
            RCLCPP_INFO(get_logger(), "小车已重置到地图起点"); }
        });

    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    // 读取启动参数 initial_map（0=环形 1=倒车入库 2=侧方停车 3=直角转弯）
    int initial_map = declare_parameter<int>("initial_map", 0);
    MapType init_type = map_type_from_int(initial_map);

    timer_ = create_wall_timer(
        std::chrono::duration<double>(SIM_DT),
        std::bind(&RingRoadSimNode::simulation_step, this));
    lidar_timer_ = create_wall_timer(
        std::chrono::milliseconds(100),
        std::bind(&RingRoadSimNode::publish_lidar, this));

    load_map(init_type);  // 按启动参数加载地图

    RCLCPP_INFO(get_logger(), "自动驾驶仿真启动：支持地图切换 + 科目二考试");
  }

private:
  // ========== 地图加载 ==========
  void load_map(MapType t) {
    map_ = create_map(t);
    driver_.reset(map_.get());
    trail_.clear();
    exam_ = ExamManager();  // 新地图重置考试
    road_markers_ = map_->build_road_markers();
    extra_markers_ = map_->build_extra_markers();
    publish_road_markers();
    RCLCPP_INFO(get_logger(), "已加载地图: %s", map_->name().c_str());
  }

  void switch_map(MapType t) {
    load_map(t);
  }

  // ========== 仿真主循环 ==========
  void simulation_step() {
    // 处理考试切换
    if (exam_.running() && exam_.consume_switch_needed()) {
      load_map(exam_.current_map_type());
    }

    if (!paused_) {
      // 环道场景注入随机障碍物（其他场景不注入，仅靠边界墙避障）
      if (map_->name() == "环形道路") {
        obstacle_manager_.update(SIM_DT);
      }
      // 把随机障碍喂给 driver 参与避障
      std::vector<Obstacle> extra;
      for (const auto& ob : obstacle_manager_.obstacles())
        extra.push_back(Obstacle{Vec2{ob.x, ob.y}, ob.radius});
      driver_.set_extra_obstacles(extra);

      // 当前目标点：考试中为科目目标，否则沿环/场景默认目标
      Vec2 target;
      bool allow_reverse = false;
      if (exam_.running()) {
        target = map_->goal_point();
        allow_reverse = (exam_.current_map_type() == MapType::kReverseParking);
      } else {
        // 非考试（含环道）：仅正向追踪默认目标点
        target = map_->goal_point();
      }

      auto res = driver_.step(target, allow_reverse, SIM_DT);

      // 记录轨迹
      sim_time_ += SIM_DT;
      trail_.push_back(make_point(driver_.car().x(), driver_.car().y(), 0.05));
      if (trail_.size() > TRAIL_MAX_POINTS) trail_.pop_front();

      // 考试完成判定
      if (exam_.running() && res.goal_reached) {
        RCLCPP_INFO(get_logger(), "科目完成: %s", exam_.current_name().c_str());
        exam_.on_item_passed();
      }

      last_front_dist_ = res.front_dist;
      last_action_ = res.action;

      if (static_cast<int>(sim_time_ * 10) % 5 == 0 &&
          static_cast<int>(sim_time_ * 100) % 100 == 0) {
        RCLCPP_INFO(get_logger(),
                    "[%.1fs] 地图:%-10s | 速度:%.2f | 前方:%.2f | 行为:%-6s",
                    sim_time_, map_->name().c_str(), res.speed, res.front_dist,
                    action_name(res.action));
      }
    }

    publish_status();
    publish_car_marker();
    publish_obstacles_marker();
    publish_lattice_marker();
    publish_car_tf();

    road_resend_accum_ += SIM_DT;
    if (road_resend_accum_ >= 2.0) {
      road_resend_accum_ = 0.0;
      publish_road_markers();
    }
  }

  // ========== 状态话题发布 ==========
  void publish_status() {
    auto f = [](double v) { auto m = std_msgs::msg::Float64(); m.data = v; return m; };
    speed_pub_->publish(f(driver_.speed()));
    action_pub_->publish(f(static_cast<double>(static_cast<int>(last_action_))));
    distance_pub_->publish(f(last_front_dist_));
    obstacle_pub2_->publish(f(static_cast<double>(obstacle_manager_.size())));

    // 考试状态
    std_msgs::msg::String status;
    status.data = exam_.status_text();
    exam_status_pub_->publish(status);

    std_msgs::msg::Int32 item;
    item.data = exam_.current_index();
    exam_item_pub_->publish(item);

    std_msgs::msg::String prog;
    char buf[64];
    std::snprintf(buf, sizeof(buf), "%d/%zu",
                  exam_.current_index() + 1, exam_.total());
    prog.data = buf;
    exam_progress_pub_->publish(prog);
  }

  // ========== 障碍物（仅环道随机障碍） ==========
  void publish_obstacles_marker() {
    visualization_msgs::msg::MarkerArray ma;
    int id = 0;
    for (const auto& ob : obstacle_manager_.obstacles()) {
      visualization_msgs::msg::Marker m;
      m.header.frame_id = "world"; m.header.stamp = now();
      m.ns = "obstacles"; m.id = id++;
      m.type = visualization_msgs::msg::Marker::CUBE; m.action = m.ADD;
      m.pose.position.x = ob.x; m.pose.position.y = ob.y; m.pose.position.z = 0.5;
      m.pose.orientation.w = 1.0;
      double sz = 2.0 * ob.radius;
      m.scale.x = sz; m.scale.y = sz; m.scale.z = 1.0;
      m.color.r = 0.95f; m.color.g = 0.25f; m.color.b = 0.1f; m.color.a = 0.95f;
      ma.markers.push_back(m);
    }
    obstacle_pub_->publish(ma);
  }

  // ========== 局部规划候选路径可视化 ==========
  void publish_lattice_marker() {
    double cx = driver_.car().x(), cy = driver_.car().y(), cyaw = driver_.car().yaw();
    auto obstacles = map_->to_obstacles();
    std::vector<LatticeTrajectory> candidates;
    lattice_planner_.plan(cx, cy, cyaw, obstacles, candidates);

    visualization_msgs::msg::MarkerArray ma;
    int id = 0;
    for (const auto& c : candidates) {
      visualization_msgs::msg::Marker m;
      m.header.frame_id = "world"; m.header.stamp = now();
      m.ns = "lattice_candidates"; m.id = id++;
      m.type = m.LINE_STRIP; m.action = m.ADD;
      m.pose.orientation.w = 1.0;
      m.scale.x = 0.06;
      if (c.selected) { m.color.r = 0; m.color.g = 1; m.color.b = 0; m.color.a = 1; m.scale.x = 0.18; }
      else            { m.color.r = 0.6f; m.color.g = 0.6f; m.color.b = 0.6f; m.color.a = 0.35f; }
      for (const auto& pt : c.path) m.points.push_back(make_point(pt.x, pt.y, 0.12));
      ma.markers.push_back(m);
    }
    plan_pub_->publish(ma);
  }

  // ========== LIDAR 点云 ==========
  void publish_lidar() {
    double cx = driver_.car().x(), cy = driver_.car().y(), cyaw = driver_.car().yaw();
    auto obstacles = map_->to_obstacles();
    // 合并随机障碍（环道）
    std::vector<Obstacle> all = obstacles;
    for (const auto& ob : obstacle_manager_.obstacles())
      all.push_back(Obstacle{Vec2{ob.x, ob.y}, ob.radius});

    sensor_msgs::msg::PointCloud2 cloud;
    cloud.header.frame_id = "world"; cloud.header.stamp = now();
    cloud.height = 1; cloud.width = LIDAR_BEAMS;
    sensor_msgs::PointCloud2Modifier mod(cloud);
    mod.setPointCloud2FieldsByString(1, "xyz");
    mod.resize(LIDAR_BEAMS);
    sensor_msgs::PointCloud2Iterator<float> it_x(cloud, "x");
    sensor_msgs::PointCloud2Iterator<float> it_y(cloud, "y");
    sensor_msgs::PointCloud2Iterator<float> it_z(cloud, "z");

    for (int i = 0; i < LIDAR_BEAMS; ++i, ++it_x, ++it_y, ++it_z) {
      double theta = cyaw + 2.0 * M_PI * i / LIDAR_BEAMS;
      double dx = std::cos(theta), dy = std::sin(theta);
      double dist = LIDAR_RANGE; double hit_t = -1.0;
      for (const auto& ob : all) {
        double ox = ob.position.x - cx, oy = ob.position.y - cy;
        double b = dx * ox + dy * oy;
        double c = ox * ox + oy * oy - ob.radius * ob.radius;
        double disc = b * b - c;
        if (disc >= 0.0) {
          double t = -b - std::sqrt(disc);
          if (t > 0.2 && t < LIDAR_RANGE && (hit_t < 0.0 || t < hit_t)) hit_t = t;
        }
      }
      if (hit_t > 0.0) dist = hit_t;
      double rr = dist + std::sin(sim_time_ * 37.0 + i * 1.3) * 0.02;
      if (rr < 0.2) rr = 0.2;
      *it_x = cx + dx * rr; *it_y = cy + dy * rr; *it_z = 0.3;
    }
    lidar_pub_->publish(cloud);
  }

  // ========== 小车可视化 ==========
  void publish_car_marker() {
    visualization_msgs::msg::MarkerArray ma;
    double cx = driver_.car().x(), cy = driver_.car().y(), cyaw = driver_.car().yaw();

    // 车体
    {
      visualization_msgs::msg::Marker m;
      m.header.frame_id = "world"; m.header.stamp = now();
      m.ns = "car"; m.id = 0;
      m.type = m.CUBE; m.action = m.ADD;
      m.pose.position.x = cx; m.pose.position.y = cy; m.pose.position.z = 0.5;
      m.pose.orientation.z = std::sin(cyaw / 2.0); m.pose.orientation.w = std::cos(cyaw / 2.0);
      m.scale.x = 1.8; m.scale.y = 0.9; m.scale.z = 0.6;
      m.color.r = 0.1f; m.color.g = 0.4f; m.color.b = 0.9f; m.color.a = 1.0f;
      ma.markers.push_back(m);
    }
    // 速度矢量
    {
      visualization_msgs::msg::Marker m;
      m.header.frame_id = "world"; m.header.stamp = now();
      m.ns = "car"; m.id = 1;
      m.type = m.ARROW; m.action = m.ADD;
      m.pose.position.x = cx; m.pose.position.y = cy; m.pose.position.z = 1.2;
      m.pose.orientation.z = std::sin(cyaw / 2.0); m.pose.orientation.w = std::cos(cyaw / 2.0);
      m.scale.x = 0.6 + std::fabs(driver_.speed()) * 0.3; m.scale.y = 0.25; m.scale.z = 0.25;
      switch (last_action_) {
        case Action::kAccelerate: m.color.r = 0; m.color.g = 1; m.color.b = 0; break;
        case Action::kCruise:     m.color.r = 0; m.color.g = 0.6f; m.color.b = 1; break;
        case Action::kBrake:      m.color.r = 1; m.color.g = 0.6f; m.color.b = 0; break;
        case Action::kStop:       m.color.r = 1; m.color.g = 0; m.color.b = 0; break;
      }
      m.color.a = 1.0f;
      ma.markers.push_back(m);
    }
    // 目标点标记（考试时高亮当前科目目标）
    if (exam_.running()) {
      Vec2 g = map_->goal_point();
      visualization_msgs::msg::Marker m;
      m.header.frame_id = "world"; m.header.stamp = now();
      m.ns = "goal"; m.id = 0;
      m.type = m.SPHERE; m.action = m.ADD;
      m.pose.position.x = g.x; m.pose.position.y = g.y; m.pose.position.z = 0.3;
      m.pose.orientation.w = 1.0;
      m.scale.x = 1.2; m.scale.y = 1.2; m.scale.z = 1.2;
      m.color.r = 0.1f; m.color.g = 1.0f; m.color.b = 0.3f; m.color.a = 0.8f;
      ma.markers.push_back(m);
    }
    // 行驶轨迹
    if (trail_.size() > 2) {
      visualization_msgs::msg::Marker m;
      m.header.frame_id = "world"; m.header.stamp = now();
      m.ns = "planning"; m.id = 1;
      m.type = m.LINE_STRIP; m.action = m.ADD;
      m.pose.orientation.w = 1.0; m.scale.x = 0.08;
      m.color.r = 0; m.color.g = 0.8f; m.color.b = 1; m.color.a = 0.6f;
      m.points.assign(trail_.begin(), trail_.end());
      ma.markers.push_back(m);
    }
    // 状态文本
    {
      visualization_msgs::msg::Marker m;
      m.header.frame_id = "world"; m.header.stamp = now();
      m.ns = "hud"; m.id = 0;
      m.type = m.TEXT_VIEW_FACING; m.action = m.ADD;
      m.pose.position.x = cx; m.pose.position.y = cy; m.pose.position.z = CAR_HEAD_Z;
      m.scale.z = 1.4;
      m.color.r = 1; m.color.g = 1; m.color.b = 1; m.color.a = 1;
      char buf[200];
      std::snprintf(buf, sizeof(buf),
                    "v=%.1f m/s  %s\n%s",
                    driver_.speed(), action_name(last_action_),
                    exam_.status_text().c_str());
      m.text = buf;
      ma.markers.push_back(m);
    }
    live_pub_->publish(ma);
  }

  // ========== 静态道路重发 ==========
  void publish_road_markers() {
    for (auto& mk : road_markers_.markers) mk.header.stamp = now();
    road_pub_->publish(road_markers_);
    // 额外标记（库位标签等）
    for (auto& mk : extra_markers_.markers) mk.header.stamp = now();
    if (!extra_markers_.markers.empty()) road_pub_->publish(extra_markers_);
  }

  // ========== TF ==========
  void publish_car_tf() {
    double cx = driver_.car().x(), cy = driver_.car().y(), cyaw = driver_.car().yaw();
    geometry_msgs::msg::TransformStamped tf;
    tf.header.stamp = now();
    tf.header.frame_id = "world";
    tf.child_frame_id = "car_base_link";
    tf.transform.translation.x = cx; tf.transform.translation.y = cy; tf.transform.translation.z = 0.0;
    tf.transform.rotation.z = std::sin(cyaw / 2.0);
    tf.transform.rotation.w = std::cos(cyaw / 2.0);
    tf_broadcaster_->sendTransform(tf);
  }

  // ========== 成员 ==========
  std::unique_ptr<ScenarioMap> map_;
  AutoDriver driver_;
  ExamManager exam_;
  ObstacleManager obstacle_manager_;

  double sim_time_{0.0};
  double road_resend_accum_{0.0};
  double last_front_dist_{LIDAR_RANGE};
  Action last_action_{Action::kCruise};
  bool   paused_{false};

  std::deque<geometry_msgs::msg::Point> trail_;
  visualization_msgs::msg::MarkerArray road_markers_;
  visualization_msgs::msg::MarkerArray extra_markers_;

  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr road_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr live_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr obstacle_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr plan_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr speed_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr action_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr distance_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr obstacle_pub2_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr exam_status_pub_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr exam_item_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr exam_progress_pub_;

  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr pause_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr clear_sub_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr algo_sub_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr set_map_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr start_exam_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr reset_sub_;

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr lidar_timer_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<RingRoadSimNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
