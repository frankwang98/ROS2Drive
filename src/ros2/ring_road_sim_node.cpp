/**
 * ring_road_sim_node.cpp — 环形道路仿真节点
 *
 * 功能：
 *   1. 用 RViz2 Marker 绘制环形道路（双车道 + 中央虚线）
 *   2. 在环道上模拟小车运动（感知 → 决策 → 控制）
 *   3. 用 Marker 可视化小车位置与朝向
 *   4. 发布 TF 坐标变换，方便在 RViz2 中查看
 *   5. 自动驾驶常用可视化：LIDAR 点云 / 规划路径 / 行驶轨迹 / 状态文本
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
#include <tf2_ros/transform_broadcaster.h>

#include "car/car.hpp"

using namespace std::chrono_literals;

// ========== 环形道路参数 ==========
constexpr double RING_RADIUS     = 25.0;   // 环道半径（米）
constexpr double ROAD_WIDTH      = 6.0;    // 道路总宽度（米）
constexpr double LANE_WIDTH      = 3.0;    // 单车道宽度（米）
constexpr int    RING_SEGMENTS   = 200;    // 环形分段数（越多越圆）
constexpr double RING_HEIGHT     = 0.05;   // 路面厚度
constexpr double LINE_HEIGHT     = 0.06;   // 标线高度
constexpr double SIM_DT          = 0.05;   // 仿真步长（秒）

// ========== 自动驾驶可视化参数 ==========
constexpr double LIDAR_RANGE     = 35.0;   // LIDAR 探测距离（米）
constexpr int    LIDAR_BEAMS     = 360;    // 水平扫描线数（1° 一根）
constexpr double PATH_LENGTH     = 30.0;   // 规划路径可视长度（米）
constexpr int    PATH_SAMPLES    = 60;     // 规划路径采样点
constexpr int    TRAIL_MAX_POINTS = 600;   // 行驶轨迹最多保留点数
constexpr double CAR_HEAD_Z      = 2.6;    // 状态文本高度

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
      , car_()
      , angle_(0.0)
      , sim_time_(0.0)
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

    // 自动驾驶常用可视化：LIDAR 点云
    lidar_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
        "sensor/lidar", live_qos);

    // HUD 面板状态话题（供自定义 RViz 面板显示）
    speed_pub_     = create_publisher<std_msgs::msg::Float64>("sdc/speed", live_qos);
    action_pub_    = create_publisher<std_msgs::msg::Float64>("sdc/action_id", live_qos);
    distance_pub_  = create_publisher<std_msgs::msg::Float64>("sdc/front_distance", live_qos);

    // 暂停/继续控制（供 HUD 面板按钮控制）
    pause_sub_ = create_subscription<std_msgs::msg::Bool>(
        "sdc/pause", 10,
        [this](const std_msgs::msg::Bool::SharedPtr msg) {
          paused_ = msg->data;
          RCLCPP_INFO(get_logger(), paused_ ? "仿真已暂停" : "仿真已继续");
        });

    // 清除行驶轨迹（供 HUD 面板按钮控制）
    clear_sub_ = create_subscription<std_msgs::msg::Bool>(
        "sdc/clear_trail", 10,
        [this](const std_msgs::msg::Bool::SharedPtr msg) {
          if (msg->data) {
            trail_.clear();
            RCLCPP_INFO(get_logger(), "行驶轨迹已清除");
          }
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

    RCLCPP_INFO(get_logger(),
                "环形道路仿真启动：半径 %.1f m, 宽度 %.1f m, 步长 %.2f s",
                RING_RADIUS, ROAD_WIDTH, SIM_DT);
  }

private:
  // ========== 仿真主循环 ==========
  void simulation_step() {
    if (!paused_) {
      //  1. 感知：在一段环形测试场景中插入一个“前方车辆”，位于当前小车前方 4~30 m
      //     这里用正弦函数模拟前车距离变化，产生加速 / 巡航 / 减速的交替行为
      car_.set_front_distance(dynamic_obstacle_distance());

      //  2. 决策 + 控制 + 物理（沿用原有 Car::step）
      double speed_before = car_.speed();
      car_.step(SIM_DT);
      double speed_after = car_.speed();
      double avg_speed = 0.5 * (speed_before + speed_after);

      //  3. 在环形道路上更新角度
      double arc_length = avg_speed * SIM_DT;
      angle_ += arc_length / RING_RADIUS;

      //  4. 更新仿真总时间，并记录轨迹
      sim_time_ += SIM_DT;
      trail_.push_back(make_point(RING_RADIUS * std::cos(angle_),
                                  RING_RADIUS * std::sin(angle_),
                                  0.05));
      if (trail_.size() > TRAIL_MAX_POINTS) {
        trail_.pop_front();
      }

      //  6. 日志（每 200 步打印一次，约 10 秒）
      if (static_cast<int>(sim_time_ * 10) % 10 == 0 &&
          static_cast<int>(sim_time_ * 100) % 100 == 0) {
        RCLCPP_INFO(get_logger(),
                    "[%.1fs] 角度:%.2f°  |  行为:%-6s  |  速度:%.2f m/s  |  距离:%.2f m",
                    sim_time_,
                    angle_ * 180.0 / M_PI,
                    sdc::action_name(car_.current_action()),
                    car_.speed(),
                    car_.front_distance());
      }
    }

    //  5. 发布状态话题 + 小车可视化
    publish_status();
    publish_car_marker();
    publish_car_tf();

    //  6. 周期性重发静态道路（每 2 秒），保证后启动的 RViz2 能看到道路
    road_resend_accum_ += SIM_DT;
    if (road_resend_accum_ >= 2.0) {
      road_resend_accum_ = 0.0;
      publish_road_markers();
    }
  }

  // ========== 状态话题发布 ==========
  void publish_status() {
    auto speed_msg = std_msgs::msg::Float64();
    speed_msg.data = car_.speed();
    speed_pub_->publish(speed_msg);

    auto action_msg = std_msgs::msg::Float64();
    action_msg.data = static_cast<double>(static_cast<int>(car_.current_action()));
    action_pub_->publish(action_msg);

    auto dist_msg = std_msgs::msg::Float64();
    dist_msg.data = car_.front_distance();
    distance_pub_->publish(dist_msg);
  }

  // ========== 动态障碍物距离模拟 ==========
  double dynamic_obstacle_distance() const {
    // 在 3~30 m 之间正弦波动，周期 ≈ 20 秒
    double base  = 16.0;
    double amp   = 13.0;
    double freq  = 2.0 * M_PI / 20.0;
    return base + amp * std::sin(freq * sim_time_);
  }

  // 小车当前位置（环道切线方向 yaw）
  void car_pose(double & x, double & y, double & yaw) const {
    x   = RING_RADIUS * std::cos(angle_);
    y   = RING_RADIUS * std::sin(angle_);
    yaw = angle_ + M_PI_2;  // 切线方向
  }

  // ========== 环形道路可视化 ==========
  // 构建静态道路 MarkerArray（缓存在 road_markers_）
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
        // 两个三角形拼成一个梯形
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
      m.scale.x = 0.15;  // 线宽
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

  // 发布静态道路
  void publish_road_markers() {
    // 刷新每个 marker 的时间戳
    for (auto& mk : road_markers_.markers) {
      mk.header.stamp = now();
    }
    road_pub_->publish(road_markers_);
  }

  // ========== LIDAR 点云 ==========
  // 模拟 2D 激光雷达：在当前小车位置，向环道边界发射 360 条射线，
  // 命中内/外边界或路面/标线即生成一个点，形成道路轮廓点云。
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

      // 射线与内/外圆环求交，取最近命中点（命中在道路范围内才有效）
      double dist = LIDAR_RANGE;
      double hit_t = -1.0;
      for (double r : {inner_r, outer_r}) {
        // 解 |p + t*d|^2 = r^2
        double b = car_x * dx + car_y * dy;
        double c = car_x * car_x + car_y * car_y - r * r;
        double disc = b * b - c;
        if (disc >= 0.0) {
          double t = -b - std::sqrt(disc);  // 最近交点（沿射线方向）
          if (t > 0.2 && t < LIDAR_RANGE && (hit_t < 0.0 || t < hit_t)) {
            hit_t = t;
          }
        }
      }
      if (hit_t > 0.0) dist = hit_t;

      // 给 LIDAR 点加一点测量噪声，更真实
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
      // 朝向：环形切线方向
      double yaw = car_yaw;
      m.pose.orientation.z = std::sin(yaw / 2.0);
      m.pose.orientation.w = std::cos(yaw / 2.0);
      m.scale.x = 1.8;  // 长
      m.scale.y = 0.9;  // 宽
      m.scale.z = 0.6;  // 高
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
      // 箭头方向 = 切线方向（速度方向）
      m.pose.orientation.z = std::sin(yaw / 2.0);
      m.pose.orientation.w = std::cos(yaw / 2.0);
      double speed = car_.speed();
      m.scale.x = 0.6 + speed * 0.3;  // 箭头长度随速度变化
      m.scale.y = 0.25;
      m.scale.z = 0.25;
      // 颜色随行为变化
      switch (car_.current_action()) {
        case sdc::Action::kAccelerate: m.color.r = 0.0f; m.color.g = 1.0f; m.color.b = 0.0f; break;
        case sdc::Action::kCruise:     m.color.r = 0.0f; m.color.g = 0.6f; m.color.b = 1.0f; break;
        case sdc::Action::kBrake:      m.color.r = 1.0f; m.color.g = 0.6f; m.color.b = 0.0f; break;
        case sdc::Action::kStop:       m.color.r = 1.0f; m.color.g = 0.0f; m.color.b = 0.0f; break;
      }
      m.color.a = 1.0f;
      ma.markers.push_back(m);
    }

    // 规划路径（绿色实线，沿环道中心向前延伸）
    {
      visualization_msgs::msg::Marker m;
      m.header.frame_id = "world";
      m.header.stamp    = now();
      m.ns    = "planning";
      m.id    = 0;
      m.type  = visualization_msgs::msg::Marker::LINE_STRIP;
      m.action = visualization_msgs::msg::Marker::ADD;
      m.pose.orientation.w = 1.0;
      m.scale.x = 0.12;  // 线宽
      m.color.r = 0.0f; m.color.g = 1.0f; m.color.b = 0.2f; m.color.a = 0.95f;

      for (int i = 0; i <= PATH_SAMPLES; ++i) {
        double s = PATH_LENGTH * i / PATH_SAMPLES;
        double a = angle_ + s / RING_RADIUS;
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
      m.scale.z = 1.4;  // 字号
      m.color.r = 1.0f; m.color.g = 1.0f; m.color.b = 1.0f; m.color.a = 1.0f;

      char buf[128];
      std::snprintf(buf, sizeof(buf),
                    "v=%.1f m/s  %s\nfront=%.1f m",
                    car_.speed(),
                    sdc::action_name(car_.current_action()),
                    car_.front_distance());
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
  sdc::Car car_;
  double   angle_;     // 小车在环道上的角度（弧度）
  double   sim_time_;  // 仿真累计时间
  double   road_resend_accum_{0.0};  // 静态道路重发计时
  bool     paused_{false};           // 暂停/继续控制

  std::deque<geometry_msgs::msg::Point> trail_;  // 行驶轨迹缓存

  visualization_msgs::msg::MarkerArray road_markers_;  // 缓存的静态道路

  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr road_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr live_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr        lidar_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr               speed_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr               action_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr               distance_pub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr               pause_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr               clear_sub_;
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
