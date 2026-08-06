/**
 * ring_road_sim_node.cpp — 环形道路仿真节点
 *
 * 功能：
 *   1. 用 RViz2 Marker 绘制环形道路（双车道 + 中央虚线）
 *   2. 在环道上模拟小车运动（感知 → 决策 → 控制）
 *   3. 用 Marker 可视化小车位置与朝向
 *   4. 发布 TF 坐标变换，方便在 RViz2 中查看
 */

#include <chrono>
#include <cmath>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
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
    // 发布器
    marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
        "simulation/markers", 10);

    // TF 广播
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    // 定时器：以 SIM_DT 频率推进仿真
    timer_ = create_wall_timer(
        std::chrono::duration<double>(SIM_DT),
        std::bind(&RingRoadSimNode::simulation_step, this));

    // 一次性绘制静态道路
    publish_road_markers();

    RCLCPP_INFO(get_logger(),
                "环形道路仿真启动：半径 %.1f m, 宽度 %.1f m, 步长 %.2f s",
                RING_RADIUS, ROAD_WIDTH, SIM_DT);
  }

private:
  // ========== 仿真主循环 ==========
  void simulation_step() {
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

    //  4. 更新仿真总时间
    sim_time_ += SIM_DT;

    //  5. 发布小车可视化
    publish_car_marker();
    publish_car_tf();

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

  // ========== 动态障碍物距离模拟 ==========
  double dynamic_obstacle_distance() const {
    // 在 3~30 m 之间正弦波动，周期 ≈ 20 秒
    double base  = 16.0;
    double amp   = 13.0;
    double freq  = 2.0 * M_PI / 20.0;
    return base + amp * std::sin(freq * sim_time_);
  }

  // ========== 环形道路可视化 ==========
  void publish_road_markers() {
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

    marker_pub_->publish(ma);
  }

  // ========== 小车可视化 ==========
  void publish_car_marker() {
    visualization_msgs::msg::MarkerArray ma;

    double car_x = RING_RADIUS * std::cos(angle_);
    double car_y = RING_RADIUS * std::sin(angle_);

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
      double yaw = angle_ + M_PI_2;  // 切线方向
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
      double yaw = angle_ + M_PI_2;
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

    marker_pub_->publish(ma);
  }

  // ========== TF 广播 ==========
  void publish_car_tf() {
    geometry_msgs::msg::TransformStamped tf;
    tf.header.stamp    = now();
    tf.header.frame_id = "world";
    tf.child_frame_id  = "car_base_link";

    tf.transform.translation.x = RING_RADIUS * std::cos(angle_);
    tf.transform.translation.y = RING_RADIUS * std::sin(angle_);
    tf.transform.translation.z = 0.0;

    double yaw = angle_ + M_PI_2;
    tf.transform.rotation.z = std::sin(yaw / 2.0);
    tf.transform.rotation.w = std::cos(yaw / 2.0);

    tf_broadcaster_->sendTransform(tf);
  }

  // ========== 成员变量 ==========
  sdc::Car car_;
  double   angle_;     // 小车在环道上的角度（弧度）
  double   sim_time_;  // 仿真累计时间

  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::TimerBase::SharedPtr                                        timer_;
  std::shared_ptr<tf2_ros::TransformBroadcaster>                     tf_broadcaster_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<RingRoadSimNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
