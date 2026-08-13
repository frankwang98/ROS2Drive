/**
 * sdc_hud_panel.hpp — 自动驾驶小车 HUD 控制面板（RViz 自定义 Panel）
 *
 * 功能：
 *   - 实时显示 速度 / 行为 / 前方距离
 *   - 切换地图（环形 / 倒车入库 / 侧方停车 / 直角转弯）
 *   - 开始科目二考试、重置小车
 *   - 暂停 / 继续 仿真、清除行驶轨迹
 *   - 显示考试状态与进度
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

#include <rclcpp/rclcpp.hpp>
#include <rviz_common/panel.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/string.hpp>

namespace sdc {

class HudPanel : public rviz_common::Panel {
  Q_OBJECT

 public:
  explicit HudPanel(QWidget* parent = nullptr);
  ~HudPanel() override;

  void onInitialize() override;
  void load(const rviz_common::Config& config) override;
  void save(rviz_common::Config config) const override;

 private Q_SLOTS:
  void onTogglePause();
  void onClearTrail();
  void onStartExam();
  void onResetCar();
  void onMapChanged(int index);
  void onStatusTimer();

 private:
  void onSpeed(const std_msgs::msg::Float64::SharedPtr msg);
  void onAction(const std_msgs::msg::Float64::SharedPtr msg);
  void onDistance(const std_msgs::msg::Float64::SharedPtr msg);
  void onExamStatus(const std_msgs::msg::String::SharedPtr msg);
  void onExamProgress(const std_msgs::msg::String::SharedPtr msg);

  // ---- 状态显示 ----
  QLabel* speed_label_;
  QLabel* action_label_;
  QLabel* distance_label_;
  QLabel* exam_label_;

  // ---- 地图选择 ----
  QComboBox* map_combo_;

  // ---- 控制按钮 ----
  QPushButton* pause_button_;
  QPushButton* clear_button_;
  QPushButton* exam_button_;
  QPushButton* reset_button_;

  // ---- UI 定时器 ----
  QTimer* ui_timer_;

  // ---- ROS ----
  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr speed_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr action_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr distance_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr exam_status_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr exam_progress_sub_;

  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pause_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr clear_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr start_exam_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr reset_pub_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr set_map_pub_;

  bool   paused_{false};
  double speed_{0.0};
  double distance_{0.0};
  int    action_id_{0};
  std::string exam_status_;
  std::string exam_progress_;
};

}  // namespace sdc

#endif  // SELF_DRIVING_CAR_RVIZ_SDC_HUD_PANEL_HPP
