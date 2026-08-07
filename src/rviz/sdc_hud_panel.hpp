/**
 * sdc_hud_panel.hpp — 自动驾驶小车 HUD 控制面板（RViz 自定义 Panel）
 *
 * 功能：
 *   - 实时显示 速度 / 行为 / 前方距离
 *   - 暂停 / 继续 仿真
 *   - 清除行驶轨迹
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
#include <QTimer>

#include <rclcpp/rclcpp.hpp>
#include <rviz_common/panel.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64.hpp>

namespace sdc {

class HudPanel : public rviz_common::Panel {
  Q_OBJECT

 public:
  explicit HudPanel(QWidget* parent = nullptr);
  ~HudPanel() override;

  // rviz_common::Panel 接口
  void onInitialize() override;
  void load(const rviz_common::Config& config) override;
  void save(rviz_common::Config config) const override;

 private Q_SLOTS:
  void onTogglePause();
  void onClearTrail();
  void onStatusTimer();

 private:
  void onSpeed(const std_msgs::msg::Float64::SharedPtr msg);
  void onAction(const std_msgs::msg::Float64::SharedPtr msg);
  void onDistance(const std_msgs::msg::Float64::SharedPtr msg);

  // ---- 状态显示 ----
  QLabel* speed_label_;
  QLabel* action_label_;
  QLabel* distance_label_;

  // ---- 控制按钮 ----
  QPushButton* pause_button_;
  QPushButton* clear_button_;

  // ---- UI 定时器 ----
  QTimer* ui_timer_;

  // ---- ROS ----
  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr speed_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr action_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr distance_sub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pause_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr clear_pub_;

  bool   paused_{false};
  double speed_{0.0};
  double distance_{0.0};
  int    action_id_{0};
};

}  // namespace sdc

#endif  // SELF_DRIVING_CAR_RVIZ_SDC_HUD_PANEL_HPP
