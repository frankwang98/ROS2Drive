/**
 * sdc_hud_panel.hpp — 自动驾驶小车 HUD 控制面板（RViz 自定义 Panel）
 *
 * 功能：
 *   - 实时显示 速度 / 行为 / 前方距离 / 驾驶模式
 *   - 自动 / 手动驾驶模式切换
 *   - 手动模式下支持 WASD 键盘控制（W=前进 S=倒车 A=左转 D=右转）
 *   - 开始 / 暂停 行驶、清除行驶轨迹、重置小车
 *
 * 通过 pluginlib 注册为 rviz_common::Panel，在 RViz 中：
 *   Panels -> Add -> New panel -> sdc/HudPanel
 */

#ifndef SELF_DRIVING_CAR_RVIZ_SDC_HUD_PANEL_HPP
#define SELF_DRIVING_CAR_RVIZ_SDC_HUD_PANEL_HPP

#include <memory>
#include <string>

#include <QWidget>
#include <QLabel>
#include <QPushButton>
#include <QComboBox>
#include <QTimer>
#include <QKeyEvent>
#include <QFrame>

#include <rclcpp/rclcpp.hpp>
#include <rviz_common/panel.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/int32.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>

namespace sdc {

class HudPanel : public rviz_common::Panel {
  Q_OBJECT

 public:
  explicit HudPanel(QWidget* parent = nullptr);
  ~HudPanel() override;

  void onInitialize() override;
  void load(const rviz_common::Config& config) override;
  void save(rviz_common::Config config) const override;

 protected:
  void keyPressEvent(QKeyEvent* event) override;
  void keyReleaseEvent(QKeyEvent* event) override;

 private Q_SLOTS:
  void onStart();
  void onTogglePause();
  void onClearTrail();
  void onToggleMode();
  void onResetCar();
  void onStatusTimer();

 private:
  void onSpeed(const std_msgs::msg::Float64::SharedPtr msg);
  void onAction(const std_msgs::msg::Float64::SharedPtr msg);
  void onDistance(const std_msgs::msg::Float64::SharedPtr msg);
  void onMode(const std_msgs::msg::Int32::SharedPtr msg);
  void onOdometry(const nav_msgs::msg::Odometry::SharedPtr msg);

  /// 发送当前手动指令（WASD 键位组合）。
  void publishManualCmd();
  /// 更新手动控制键位状态（按下/松开）。
  void updateKey(int key, bool pressed);

  // ---- 状态显示 ----
  QLabel* speed_label_;
  QLabel* action_label_;
  QLabel* distance_label_;
  QLabel* mode_label_;
  QFrame* viewport_hud_{nullptr};
  QLabel* viewport_title_{nullptr};
  QLabel* viewport_speed_{nullptr};
  QLabel* viewport_pose_{nullptr};

  // ---- 控制按钮 ----
  QPushButton* start_button_;
  QPushButton* pause_button_;
  QPushButton* clear_button_;
  QPushButton* mode_button_;
  QPushButton* reset_button_;

  // ---- UI 定时器 ----
  QTimer* ui_timer_;

  // ---- ROS ----
  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr speed_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr action_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr distance_sub_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr mode_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr start_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pause_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr clear_pub_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr set_mode_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr reset_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr manual_pub_;

  // ---- 驾驶状态 ----
  bool   paused_{false};
  bool   manual_{false};        // 当前是否为手动模式
  double speed_{0.0};
  double distance_{0.0};
  double pose_x_{0.0};
  double pose_y_{0.0};
  double pose_z_{0.0};
  int    action_id_{0};

  // WASD 键位状态
  bool key_w_{false};  // 前进
  bool key_s_{false};  // 倒车
  bool key_a_{false};  // 左转
  bool key_d_{false};  // 右转
};

}  // namespace sdc

#endif  // SELF_DRIVING_CAR_RVIZ_SDC_HUD_PANEL_HPP
