/**
 * ring_road_sim_node.cpp — 环形道路仿真节点（含避障 + 控制算法 + RViz 可视化）
 *
 * 功能：
 *   1. 用 RViz2 Marker 绘制环形道路（双车道 + 中央虚线）
 *   2. 随机生成障碍物，并在 RViz 中以红色方块显示
 *   3. 基于 Lattice 采样做局部规划避障，并可视化候选路径 / 最优路径
 *   4. PID / Bang-Bang / Ramp 等多种速度控制算法（可通过话题切换）
 *   5. 感知 → 决策 → 控制 闭环，小车沿环道避障行驶
 *   6. 发布 TF 坐标变换、LIDAR、规划路径、状态文本等
 */

#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <deque>
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
#include <tf2_ros/transform_broadcaster.h>

#include "control/velocity_controller.hpp"
#include "control/steering_controller.hpp"
#include "decision/decision_maker.hpp"
#include "model/ackermann_model.hpp"
#include "planning/lattice_planner.hpp"
#include "sim/obstacle_manager.hpp"

using namespace std::chrono_literals;

// ========== 环形道路参数 ==========
constexpr double RING_RADIUS     = 25.0;   // 环道半径（米）
constexpr double ROAD_WIDTH      = 6.0;    // 道路总宽度（米）
constexpr double LANE_WIDTH      = 3.0;    // 单车道宽度（米）
constexpr int    RING_SEGMENTS   = 200;    // 环形分段数（越多越圆）
constexpr double RING_HEIGHT     = 0.05;   // 路面厚度
constexpr double LINE_HEIGHT     = 0.06;   // 标线高度
constexpr double SIM_DT          = 0.05;   // 仿真步长（秒）

// ========== 阿克曼转向控制参数 ==========
constexpr double LOOKAHEAD      = 6.0;    // 转向目标点的前视距离（米）

// ========== 自动驾驶可视化参数 ==========
constexpr double LIDAR_RANGE     = 35.0;   // LIDAR 探测距离（米）
constexpr int    LIDAR_BEAMS     = 360;    // 水平扫描线数（1° 一根）
constexpr double PATH_LENGTH     = 30.0;   // 规划路径可视长度（米）
constexpr int    PATH_SAMPLES    = 60;     // 规划路径采样点
constexpr int    TRAIL_MAX_POINTS = 600;   // 行驶轨迹最多保留点数
constexpr double CAR_HEAD_Z      = 2.6;    // 状态文本高度

// ========== 决策目标速度 ==========
static double target_speed_for_action(sdc::Action a) {
  switch (a) {
    case sdc::Action::kAccelerate: return 3.0;
    case sdc::Action::kCruise:     return 2.0;
    case sdc::Action::kBrake:      return 0.5;
    case sdc::Action::kStop:       return 0.0;
    default:                       return 0.0;
  }
}

// ========== 工具函数 ==========
static geometry_msgs::msg::Point make_point(double x, double y, double z = 0.0) {
  geometry_msgs::msg::Point p;
  p.x = x; p.y = y; p.z = z;
  return p;
}

class RingRoadSimNode : public rclcpp::Node {
public:
  RingRoadSimNode()
      : Node("ring_road_sim")
      , sim_time_(0.0)
      , obstacle_manager_(RING_RADIUS, ROAD_WIDTH, 20240812)
      , lattice_planner_()
      , velocity_controller_()
      , steering_controller_()
      , ackermann_()
  {
    // 发布器：
    //  - 静态道路使用 transient_local + reliable，
    //    与 RViz2 MarkerArray 默认的 Reliable 订阅兼容，保证后启动的 RViz 打开即可收到。
    //  - 动态内容使用 reliable（与 RViz 默认一致）。
    auto road_qos = rclcpp::QoS(10).transient_local();
    auto live_qos = rclcpp::QoS(10).reliable();
    road_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
        "simulation/markers", road_qos);
    live_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
        "simulation/markers_live", live_qos);
    // 障碍物可视化（独立话题，方便在 RViz 中单独开关）
    obstacle_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
        "simulation/obstacles", live_qos);
    // 局部规划候选路径可视化
    plan_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
        "simulation/lattice", live_qos);

    // 自动驾驶常用可视化：LIDAR 点云
    lidar_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
        "sensor/lidar", live_qos);

    // HUD 面板状态话题
    speed_pub_     = create_publisher<std_msgs::msg::Float64>("sdc/speed", live_qos);
    action_pub_    = create_publisher<std_msgs::msg::Float64>("sdc/action_id", live_qos);
    distance_pub_  = create_publisher<std_msgs::msg::Float64>("sdc/front_distance", live_qos);
    obstacle_pub2_ = create_publisher<std_msgs::msg::Float64>("sdc/obstacle_count", live_qos);

    // 暂停/继续控制
    pause_sub_ = create_subscription<std_msgs::msg::Bool>(
        "sdc/pause", 10,
        [this](const std_msgs::msg::Bool::SharedPtr msg) {
          paused_ = msg->data;
          RCLCPP_INFO(get_logger(), paused_ ? "仿真已暂停" : "仿真已继续");
        });

    // 清除行驶轨迹
    clear_sub_ = create_subscription<std_msgs::msg::Bool>(
        "sdc/clear_trail", 10,
        [this](const std_msgs::msg::Bool::SharedPtr msg) {
          if (msg->data) {
            trail_.clear();
            RCLCPP_INFO(get_logger(), "行驶轨迹已清除");
          }
        });

    // 切换速度控制算法（0=PID 1=Bang-Bang 2=Ramp）
    algo_sub_ = create_subscription<std_msgs::msg::Int32>(
        "sdc/control_algo", 10,
        [this](const std_msgs::msg::Int32::SharedPtr msg) {
          int a = msg->data;
          if (a == 1)      velocity_controller_.set_algorithm(sdc::VelocityAlgorithm::kBangBang);
          else if (a == 2) velocity_controller_.set_algorithm(sdc::VelocityAlgorithm::kRamp);
          else             velocity_controller_.set_algorithm(sdc::VelocityAlgorithm::kPid);
          RCLCPP_INFO(get_logger(), "控制算法切换为: %s",
                      sdc::velocity_algorithm_name(velocity_controller_.algorithm()));
        });

    // TF 广播
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    // 定时器：以 SIM_DT 频率推进仿真
    timer_ = create_wall_timer(
        std::chrono::duration<double>(SIM_DT),
        std::bind(&RingRoadSimNode::simulation_step, this));

    // 一次性绘制静态道路并缓存
    road_markers_ = build_road_markers();
    publish_road_markers();

    // 定时器：以 10Hz 发布 LIDAR 点云
    lidar_timer_ = create_wall_timer(
        std::chrono::milliseconds(100),
        std::bind(&RingRoadSimNode::publish_lidar, this));

    // 初始化阿克曼运动学模型：起点位于环道东侧（角度 0），朝向切线方向（π/2）
    ackermann_.reset(RING_RADIUS, 0.0, M_PI_2, 0.0, 0.0);

    RCLCPP_INFO(get_logger(),
                "环形道路仿真启动：半径 %.1f m, 宽度 %.1f m, 步长 %.2f s, 控制算法=%s, 运动学=阿克曼单车模型",
                RING_RADIUS, ROAD_WIDTH, SIM_DT,
                sdc::velocity_algorithm_name(velocity_controller_.algorithm()));
  }

private:
  // ========== 仿真主循环 ==========
  void simulation_step() {
    if (!paused_) {
      //  1. 更新随机障碍物
      obstacle_manager_.update(SIM_DT);

      //  2. 感知：计算小车前方最近障碍物距离
      double front_dist = front_obstacle_distance();

      //  3. 局部规划：lattice planner 生成候选轨迹并选择最优避障路径
      double car_x, car_y, car_yaw;
      car_pose(car_x, car_y, car_yaw);
      auto obstacles = obstacle_manager_.to_planner_obstacles();
      std::vector<sdc::LatticeTrajectory> candidates;
      lattice_planner_.plan(car_x, car_y, car_yaw, obstacles, candidates);

      //  4. 决策：根据前方距离决定目标速度
      sdc::Action action = decision_maker_.decide(front_dist);
      double target_speed = target_speed_for_action(action);

      //  5. 控制：PID/其他算法闭环控制速度
      speed_ = velocity_controller_.update(target_speed, speed_, SIM_DT);

      //  6. 阿克曼运动学更新：基于转向控制 + 单车模型，真实转弯行驶
      //  6.1 确定目标横向偏移（避障后的期望车道位置）
      double target_lateral = 0.0;
      for (const auto& c : candidates) {
        if (c.selected) {
          target_lateral = c.lateral_offset;
          break;
        }
      }
      //  6.2 沿环道取前视目标点（含目标横向偏移），作为转向跟踪的期望路径点
      double car_x2, car_y2, car_yaw2;
      car_pose(car_x2, car_y2, car_yaw2);
      double goal_angle = angle_of_pose(car_x2, car_y2);
      double goal_a = goal_angle + LOOKAHEAD / RING_RADIUS;
      double goal_x = (RING_RADIUS + target_lateral) * std::cos(goal_a);
      double goal_y = (RING_RADIUS + target_lateral) * std::sin(goal_a);

      //  6.3 Stanley 转向控制 → 前轮转角
      double steer = steering_controller_.compute(
          car_x2, car_y2, car_yaw2, goal_x, goal_y, speed_);

      //  6.4 单车运动学积分（含转向限幅与转向速率限制，抑制"画龙"）
      ackermann_.update(speed_, steer, SIM_DT);

      //  7. 记录轨迹（基于运动学模型的实际车体位置）
      sim_time_ += SIM_DT;
      double tx, ty, tyaw;
      car_pose(tx, ty, tyaw);
      trail_.push_back(make_point(tx, ty, 0.05));
      if (trail_.size() > TRAIL_MAX_POINTS) {
        trail_.pop_front();
      }

      //  8. 日志（约每 5 秒）
      if (static_cast<int>(sim_time_ * 10) % 5 == 0 &&
          static_cast<int>(sim_time_ * 100) % 100 == 0) {
        RCLCPP_INFO(get_logger(),
                    "[%.1fs] 行为:%-6s | 速度:%.2f | 前方:%.2f | 障碍物:%zu | 横向:%.2f | 转角:%.2f°",
                    sim_time_,
                    sdc::action_name(action),
                    speed_,
                    front_dist,
                    obstacle_manager_.size(),
                    lateral_offset(tx, ty),
                    ackermann_.steer() * 180.0 / M_PI);
      }
    }

    //  9. 发布状态话题 + 可视化
    publish_status();
    publish_car_marker();
    publish_obstacles_marker();
    publish_lattice_marker();
    publish_car_tf();

    //  10. 周期性重发静态道路（每 2 秒），保证后启动的 RViz2 能看到道路
    road_resend_accum_ += SIM_DT;
    if (road_resend_accum_ >= 2.0) {
      road_resend_accum_ = 0.0;
      publish_road_markers();
    }
  }

  // ========== 状态话题发布 ==========
  void publish_status() {
    auto speed_msg = std_msgs::msg::Float64();
    speed_msg.data = speed_;
    speed_pub_->publish(speed_msg);

    auto action_msg = std_msgs::msg::Float64();
    action_msg.data = static_cast<double>(static_cast<int>(decision_maker_.decide(front_obstacle_distance())));
    action_pub_->publish(action_msg);

    auto dist_msg = std_msgs::msg::Float64();
    dist_msg.data = front_obstacle_distance();
    distance_pub_->publish(dist_msg);

    auto ob_msg = std_msgs::msg::Float64();
    ob_msg.data = static_cast<double>(obstacle_manager_.size());
    obstacle_pub2_->publish(ob_msg);
  }

  // ========== 前方障碍物距离感知 ==========
  double front_obstacle_distance() const {
    double car_x, car_y, car_yaw;
    car_pose(car_x, car_y, car_yaw);
    double fx = std::cos(car_yaw);  // 前进方向
    double fy = std::sin(car_yaw);

    double min_d = LIDAR_RANGE;
    for (const auto& ob : obstacle_manager_.obstacles()) {
      double dx = ob.x - car_x;
      double dy = ob.y - car_y;
      double dist = std::sqrt(dx * dx + dy * dy) - ob.radius;
      // 只考虑小车前方的障碍物
      if (dist < 0) dist = 0.0;
      if (dist < min_d && (dx * fx + dy * fy) > 0.2) {
        min_d = dist;
      }
    }
    return min_d;
  }

  // 小车当前位置：直接来自阿克曼运动学模型（x, y 为世界坐标，yaw 为朝向）
  void car_pose(double & x, double & y, double & yaw) const {
    x   = ackermann_.x();
    y   = ackermann_.y();
    yaw = ackermann_.yaw();
  }

  // 将小车位置映射回环道对应的角度（用于计算前视目标点）
  static double angle_of_pose(double x, double y) {
    return std::atan2(y, x);
  }

  // 小车相对环道中心线的横向偏移（世界半径 - 环道半径）
  static double lateral_offset(double x, double y) {
    return std::hypot(x, y) - RING_RADIUS;
  }

  // ========== 环形道路可视化 ==========
  visualization_msgs::msg::MarkerArray build_road_markers() {
    visualization_msgs::msg::MarkerArray ma;
    // --- 路面（灰色半透明圆环） ---
    {
      visualization_msgs::msg::Marker m;
      m.header.frame_id = "world";
      m.header.stamp    = now();
      m.ns    = "road";
      m.id    = 0;
      m.type  = visualization_msgs::msg::Marker::TRIANGLE_LIST;
      m.action = visualization_msgs::msg::Marker::ADD;
      m.pose.orientation.w = 1.0;
      m.scale.x = 1.0; m.scale.y = 1.0; m.scale.z = 1.0;
      m.color.r = 0.2f; m.color.g = 0.2f; m.color.b = 0.25f; m.color.a = 0.85f;

      double inner_r = RING_RADIUS - ROAD_WIDTH / 2.0;
      double outer_r = RING_RADIUS + ROAD_WIDTH / 2.0;
      for (int i = 0; i < RING_SEGMENTS; ++i) {
        double a0 = 2.0 * M_PI * i / RING_SEGMENTS;
        double a1 = 2.0 * M_PI * (i + 1) / RING_SEGMENTS;
        m.points.push_back(make_point(inner_r * std::cos(a0), inner_r * std::sin(a0), 0.0));
        m.points.push_back(make_point(outer_r * std::cos(a0), outer_r * std::sin(a0), 0.0));
        m.points.push_back(make_point(outer_r * std::cos(a1), outer_r * std::sin(a1), 0.0));

        m.points.push_back(make_point(inner_r * std::cos(a0), inner_r * std::sin(a0), 0.0));
        m.points.push_back(make_point(outer_r * std::cos(a1), outer_r * std::sin(a1), 0.0));
        m.points.push_back(make_point(inner_r * std::cos(a1), inner_r * std::sin(a1), 0.0));
      }
      ma.markers.push_back(m);
    }

    // --- 中央虚线 ---
    {
      visualization_msgs::msg::Marker m;
      m.header.frame_id = "world";
      m.header.stamp    = now();
      m.ns    = "road";
      m.id    = 1;
      m.type  = visualization_msgs::msg::Marker::LINE_LIST;
      m.action = visualization_msgs::msg::Marker::ADD;
      m.pose.orientation.w = 1.0;
      m.scale.x = 0.15;
      m.color.r = 1.0f; m.color.g = 1.0f; m.color.b = 1.0f; m.color.a = 0.9f;

      constexpr int DASH_COUNT = 100;
      for (int i = 0; i < DASH_COUNT; ++i) {
        double a0 = 2.0 * M_PI * i / DASH_COUNT;
        double a1 = 2.0 * M_PI * (i + 0.5) / DASH_COUNT;
        double z  = LINE_HEIGHT;
        m.points.push_back(make_point(RING_RADIUS * std::cos(a0), RING_RADIUS * std::sin(a0), z));
        m.points.push_back(make_point(RING_RADIUS * std::cos(a1), RING_RADIUS * std::sin(a1), z));
      }
      ma.markers.push_back(m);
    }

    // --- 内边界（白色实线） ---
    {
      visualization_msgs::msg::Marker m;
      m.header.frame_id = "world";
      m.header.stamp    = now();
      m.ns    = "road";
      m.id    = 2;
      m.type  = visualization_msgs::msg::Marker::LINE_STRIP;
      m.action = visualization_msgs::msg::Marker::ADD;
      m.pose.orientation.w = 1.0;
      m.scale.x = 0.25;
      m.color.r = 1.0f; m.color.g = 1.0f; m.color.b = 1.0f; m.color.a = 0.95f;

      double r = RING_RADIUS - ROAD_WIDTH / 2.0;
      for (int i = 0; i <= RING_SEGMENTS; ++i) {
        double a = 2.0 * M_PI * i / RING_SEGMENTS;
        m.points.push_back(make_point(r * std::cos(a), r * std::sin(a), LINE_HEIGHT));
      }
      ma.markers.push_back(m);
    }

    // --- 外边界（白色实线） ---
    {
      visualization_msgs::msg::Marker m;
      m.header.frame_id = "world";
      m.header.stamp    = now();
      m.ns    = "road";
      m.id    = 3;
      m.type  = visualization_msgs::msg::Marker::LINE_STRIP;
      m.action = visualization_msgs::msg::Marker::ADD;
      m.pose.orientation.w = 1.0;
      m.scale.x = 0.25;
      m.color.r = 1.0f; m.color.g = 1.0f; m.color.b = 1.0f; m.color.a = 0.95f;

      double r = RING_RADIUS + ROAD_WIDTH / 2.0;
      for (int i = 0; i <= RING_SEGMENTS; ++i) {
        double a = 2.0 * M_PI * i / RING_SEGMENTS;
        m.points.push_back(make_point(r * std::cos(a), r * std::sin(a), LINE_HEIGHT));
      }
      ma.markers.push_back(m);
    }

    return ma;
  }

  void publish_road_markers() {
    for (auto& mk : road_markers_.markers) {
      mk.header.stamp = now();
    }
    road_pub_->publish(road_markers_);
  }

  // ========== 障碍物可视化 ==========
  void publish_obstacles_marker() {
    visualization_msgs::msg::MarkerArray ma;
    int id = 0;
    for (const auto& ob : obstacle_manager_.obstacles()) {
      visualization_msgs::msg::Marker m;
      m.header.frame_id = "world";
      m.header.stamp    = now();
      m.ns    = "obstacles";
      m.id    = id++;
      m.type  = visualization_msgs::msg::Marker::CUBE;
      m.action = visualization_msgs::msg::Marker::ADD;
      m.pose.position.x = ob.x;
      m.pose.position.y = ob.y;
      m.pose.position.z = 0.5;
      m.pose.orientation.w = 1.0;
      double sz = 2.0 * ob.radius;
      m.scale.x = sz;
      m.scale.y = sz;
      m.scale.z = 1.0;
      // 红/橙障碍物
      m.color.r = 0.95f; m.color.g = 0.25f; m.color.b = 0.1f; m.color.a = 0.95f;
      ma.markers.push_back(m);
    }
    obstacle_pub_->publish(ma);
  }

  // ========== 局部规划候选路径可视化 ==========
  void publish_lattice_marker() {
    double car_x, car_y, car_yaw;
    car_pose(car_x, car_y, car_yaw);
    auto obstacles = obstacle_manager_.to_planner_obstacles();
    std::vector<sdc::LatticeTrajectory> candidates;
    lattice_planner_.plan(car_x, car_y, car_yaw, obstacles, candidates);

    visualization_msgs::msg::MarkerArray ma;
    int id = 0;
    for (const auto& c : candidates) {
      visualization_msgs::msg::Marker m;
      m.header.frame_id = "world";
      m.header.stamp    = now();
      m.ns    = "lattice_candidates";
      m.id    = id++;
      m.type  = visualization_msgs::msg::Marker::LINE_STRIP;
      m.action = visualization_msgs::msg::Marker::ADD;
      m.pose.orientation.w = 1.0;
      m.scale.x = 0.06;
      // 选中的最优路径高亮为绿色，其余候选为灰色
      if (c.selected) {
        m.color.r = 0.0f; m.color.g = 1.0f; m.color.b = 0.0f; m.color.a = 1.0f;
        m.scale.x = 0.18;
      } else {
        m.color.r = 0.6f; m.color.g = 0.6f; m.color.b = 0.6f; m.color.a = 0.35f;
      }
      for (const auto& pt : c.path) {
        m.points.push_back(make_point(pt.x, pt.y, 0.12));
      }
      ma.markers.push_back(m);
    }
    plan_pub_->publish(ma);
  }

  // ========== LIDAR 点云 ==========
  void publish_lidar() {
    double car_x, car_y, car_yaw;
    car_pose(car_x, car_y, car_yaw);

    const double inner_r = RING_RADIUS - ROAD_WIDTH / 2.0;
    const double outer_r = RING_RADIUS + ROAD_WIDTH / 2.0;

    sensor_msgs::msg::PointCloud2 cloud;
    cloud.header.frame_id = "world";
    cloud.header.stamp    = now();
    cloud.height = 1;
    cloud.width  = LIDAR_BEAMS;

    sensor_msgs::PointCloud2Modifier mod(cloud);
    mod.setPointCloud2FieldsByString(1, "xyz");
    mod.resize(LIDAR_BEAMS);

    sensor_msgs::PointCloud2Iterator<float> it_x(cloud, "x");
    sensor_msgs::PointCloud2Iterator<float> it_y(cloud, "y");
    sensor_msgs::PointCloud2Iterator<float> it_z(cloud, "z");

    for (int i = 0; i < LIDAR_BEAMS; ++i, ++it_x, ++it_y, ++it_z) {
      double theta = car_yaw + 2.0 * M_PI * i / LIDAR_BEAMS;
      double dx = std::cos(theta);
      double dy = std::sin(theta);

      // 射线与内/外圆环求交，取最近命中点
      double dist = LIDAR_RANGE;
      double hit_t = -1.0;
      for (double r : {inner_r, outer_r}) {
        double b = car_x * dx + car_y * dy;
        double c = car_x * car_x + car_y * car_y - r * r;
        double disc = b * b - c;
        if (disc >= 0.0) {
          double t = -b - std::sqrt(disc);
          if (t > 0.2 && t < LIDAR_RANGE && (hit_t < 0.0 || t < hit_t)) {
            hit_t = t;
          }
        }
      }
      // 障碍物也会被 LIDAR 探测到：命中最近的障碍物
      for (const auto& ob : obstacle_manager_.obstacles()) {
        // 射线与圆（障碍物）求交
        double ox = ob.x - car_x, oy = ob.y - car_y;
        double b = dx * ox + dy * oy;
        double c = ox * ox + oy * oy - ob.radius * ob.radius;
        double disc = b * b - c;
        if (disc >= 0.0) {
          double t = -b - std::sqrt(disc);
          if (t > 0.2 && t < LIDAR_RANGE && (hit_t < 0.0 || t < hit_t)) {
            hit_t = t;
          }
        }
      }
      if (hit_t > 0.0) dist = hit_t;

      double rr = dist + std::sin(sim_time_ * 37.0 + i * 1.3) * 0.02;
      if (rr < 0.2) rr = 0.2;

      *it_x = car_x + dx * rr;
      *it_y = car_y + dy * rr;
      *it_z = 0.3;
    }

    lidar_pub_->publish(cloud);
  }

  // ========== 小车可视化 ==========
  void publish_car_marker() {
    visualization_msgs::msg::MarkerArray ma;

    double car_x, car_y, car_yaw;
    car_pose(car_x, car_y, car_yaw);

    // 车体（蓝色立方体）
    {
      visualization_msgs::msg::Marker m;
      m.header.frame_id = "world";
      m.header.stamp    = now();
      m.ns    = "car";
      m.id    = 0;
      m.type  = visualization_msgs::msg::Marker::CUBE;
      m.action = visualization_msgs::msg::Marker::ADD;
      m.pose.position.x = car_x;
      m.pose.position.y = car_y;
      m.pose.position.z = 0.5;
      double yaw = car_yaw;
      m.pose.orientation.z = std::sin(yaw / 2.0);
      m.pose.orientation.w = std::cos(yaw / 2.0);
      m.scale.x = 1.8;
      m.scale.y = 0.9;
      m.scale.z = 0.6;
      m.color.r = 0.1f; m.color.g = 0.4f; m.color.b = 0.9f; m.color.a = 1.0f;
      ma.markers.push_back(m);
    }

    // 速度矢量（根据行为变色）
    {
      visualization_msgs::msg::Marker m;
      m.header.frame_id = "world";
      m.header.stamp    = now();
      m.ns    = "car";
      m.id    = 1;
      m.type  = visualization_msgs::msg::Marker::ARROW;
      m.action = visualization_msgs::msg::Marker::ADD;
      m.pose.position.x = car_x;
      m.pose.position.y = car_y;
      m.pose.position.z = 1.2;
      double yaw = car_yaw;
      m.pose.orientation.z = std::sin(yaw / 2.0);
      m.pose.orientation.w = std::cos(yaw / 2.0);
      m.scale.x = 0.6 + speed_ * 0.3;
      m.scale.y = 0.25;
      m.scale.z = 0.25;
      switch (decision_maker_.decide(front_obstacle_distance())) {
        case sdc::Action::kAccelerate: m.color.r = 0.0f; m.color.g = 1.0f; m.color.b = 0.0f; break;
        case sdc::Action::kCruise:     m.color.r = 0.0f; m.color.g = 0.6f; m.color.b = 1.0f; break;
        case sdc::Action::kBrake:      m.color.r = 1.0f; m.color.g = 0.6f; m.color.b = 0.0f; break;
        case sdc::Action::kStop:       m.color.r = 1.0f; m.color.g = 0.0f; m.color.b = 0.0f; break;
      }
      m.color.a = 1.0f;
      ma.markers.push_back(m);
    }

    // 规划路径（沿环道中心前伸，绿色实线）
    {
      visualization_msgs::msg::Marker m;
      m.header.frame_id = "world";
      m.header.stamp    = now();
      m.ns    = "planning";
      m.id    = 0;
      m.type  = visualization_msgs::msg::Marker::LINE_STRIP;
      m.action = visualization_msgs::msg::Marker::ADD;
      m.pose.orientation.w = 1.0;
      m.scale.x = 0.12;
      m.color.r = 0.0f; m.color.g = 1.0f; m.color.b = 0.2f; m.color.a = 0.95f;

      double base_a = angle_of_pose(car_x, car_y);
      for (int i = 0; i <= PATH_SAMPLES; ++i) {
        double s = PATH_LENGTH * i / PATH_SAMPLES;
        double a = base_a + s / RING_RADIUS;
        m.points.push_back(make_point(RING_RADIUS * std::cos(a),
                                      RING_RADIUS * std::sin(a),
                                      0.15));
      }
      ma.markers.push_back(m);
    }

    // 行驶轨迹（青色渐变，历史路径）
    if (trail_.size() > 2) {
      visualization_msgs::msg::Marker m;
      m.header.frame_id = "world";
      m.header.stamp    = now();
      m.ns    = "planning";
      m.id    = 1;
      m.type  = visualization_msgs::msg::Marker::LINE_STRIP;
      m.action = visualization_msgs::msg::Marker::ADD;
      m.pose.orientation.w = 1.0;
      m.scale.x = 0.08;
      m.color.r = 0.0f; m.color.g = 0.8f; m.color.b = 1.0f; m.color.a = 0.6f;
      m.points.assign(trail_.begin(), trail_.end());
      ma.markers.push_back(m);
    }

    // 自动驾驶状态文本（3D 文本，悬于车上方）
    {
      visualization_msgs::msg::Marker m;
      m.header.frame_id = "world";
      m.header.stamp    = now();
      m.ns    = "hud";
      m.id    = 0;
      m.type  = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
      m.action = visualization_msgs::msg::Marker::ADD;
      m.pose.position.x = car_x;
      m.pose.position.y = car_y;
      m.pose.position.z = CAR_HEAD_Z;
      m.scale.z = 1.4;
      m.color.r = 1.0f; m.color.g = 1.0f; m.color.b = 1.0f; m.color.a = 1.0f;

      char buf[160];
      std::snprintf(buf, sizeof(buf),
                    "v=%.1f m/s  %s\nfront=%.1f m  algo=%s",
                    speed_,
                    sdc::action_name(decision_maker_.decide(front_obstacle_distance())),
                    front_obstacle_distance(),
                    sdc::velocity_algorithm_name(velocity_controller_.algorithm()));
      m.text = buf;
      ma.markers.push_back(m);
    }

    live_pub_->publish(ma);
  }

  // ========== TF 广播 ==========
  void publish_car_tf() {
    double car_x, car_y, car_yaw;
    car_pose(car_x, car_y, car_yaw);

    geometry_msgs::msg::TransformStamped tf;
    tf.header.stamp    = now();
    tf.header.frame_id = "world";
    tf.child_frame_id  = "car_base_link";

    tf.transform.translation.x = car_x;
    tf.transform.translation.y = car_y;
    tf.transform.translation.z = 0.0;

    tf.transform.rotation.z = std::sin(car_yaw / 2.0);
    tf.transform.rotation.w = std::cos(car_yaw / 2.0);

    tf_broadcaster_->sendTransform(tf);
  }

  // ========== 成员变量 ==========
  sdc::DecisionMaker decision_maker_;
  sdc::VelocityController velocity_controller_;
  sdc::SteeringController steering_controller_;
  sdc::ObstacleManager obstacle_manager_;
  sdc::LatticePlanner lattice_planner_;
  sdc::AckermannModel ackermann_;

  double   speed_{0.0};      // 小车当前速度（m/s）
  double   sim_time_{0.0};   // 仿真累计时间
  double   road_resend_accum_{0.0};
  bool     paused_{false};

  std::deque<geometry_msgs::msg::Point> trail_;
  visualization_msgs::msg::MarkerArray road_markers_;

  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr road_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr live_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr obstacle_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr plan_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr        lidar_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr               speed_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr               action_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr               distance_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr               obstacle_pub2_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr               pause_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr               clear_sub_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr              algo_sub_;
  rclcpp::TimerBase::SharedPtr                                       timer_;
  rclcpp::TimerBase::SharedPtr                                       lidar_timer_;
  std::shared_ptr<tf2_ros::TransformBroadcaster>                     tf_broadcaster_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<RingRoadSimNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
