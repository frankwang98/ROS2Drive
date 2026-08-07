/**
 * sdc_hud_panel.cpp — 自动驾驶小车 HUD 控制面板实现
 */

#include "sdc_hud_panel.hpp"

#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QGroupBox>
#include <QString>

#include <rviz_common/display_context.hpp>

namespace sdc {

static const char* kActionNames[] = {"加速", "巡航", "减速", "停车"};

HudPanel::HudPanel(QWidget* parent)
    : rviz_common::Panel(parent)
    , speed_label_(nullptr)
    , action_label_(nullptr)
    , distance_label_(nullptr)
    , pause_button_(nullptr)
    , clear_button_(nullptr)
    , ui_timer_(nullptr)
{
  // ---------- 界面布局 ----------
  auto* root = new QVBoxLayout(this);

  // 状态区
  auto* state_box = new QGroupBox("小车状态", this);
  auto* state_layout = new QVBoxLayout(state_box);
  speed_label_ = new QLabel("速度: -- m/s", state_box);
  action_label_ = new QLabel("行为: --", state_box);
  distance_label_ = new QLabel("前方距离: -- m", state_box);
  state_layout->addWidget(speed_label_);
  state_layout->addWidget(action_label_);
  state_layout->addWidget(distance_label_);
  root->addWidget(state_box);

  // 控制区
  auto* ctrl_box = new QGroupBox("控制", this);
  auto* ctrl_layout = new QHBoxLayout(ctrl_box);
  pause_button_ = new QPushButton("暂停", ctrl_box);
  clear_button_ = new QPushButton("清除轨迹", ctrl_box);
  ctrl_layout->addWidget(pause_button_);
  ctrl_layout->addWidget(clear_button_);
  root->addWidget(ctrl_box);

  // 连接信号
  connect(pause_button_, &QPushButton::clicked, this, &HudPanel::onTogglePause);
  connect(clear_button_, &QPushButton::clicked, this, &HudPanel::onClearTrail);

  // 更新速度表样式（大号字体）
  QFont font = speed_label_->font();
  font.setPointSize(14);
  font.setBold(true);
  speed_label_->setFont(font);
}

void HudPanel::onInitialize() {
  // 使用 RViz 自身的 ROS 节点（由 RViz 负责 spin），避免额外线程
  if (!getDisplayContext()) return;
  // Humble/Jazzy 中 getRosNodeAbstraction() 返回 weak_ptr，需 lock() 提升为 shared_ptr
  auto ros_node_abs = getDisplayContext()->getRosNodeAbstraction().lock();
  if (!ros_node_abs) return;
  node_ = ros_node_abs->get_raw_node();

  auto qos = rclcpp::QoS(10);
  speed_sub_ = node_->create_subscription<std_msgs::msg::Float64>(
      "sdc/speed", qos, std::bind(&HudPanel::onSpeed, this, std::placeholders::_1));
  action_sub_ = node_->create_subscription<std_msgs::msg::Float64>(
      "sdc/action_id", qos, std::bind(&HudPanel::onAction, this, std::placeholders::_1));
  distance_sub_ = node_->create_subscription<std_msgs::msg::Float64>(
      "sdc/front_distance", qos, std::bind(&HudPanel::onDistance, this, std::placeholders::_1));

  pause_pub_ = node_->create_publisher<std_msgs::msg::Bool>("sdc/pause", qos);
  clear_pub_ = node_->create_publisher<std_msgs::msg::Bool>("sdc/clear_trail", qos);

  // 定时刷新界面（Qt 主线程）
  ui_timer_ = new QTimer(this);
  ui_timer_->setInterval(100);
  connect(ui_timer_, &QTimer::timeout, this, &HudPanel::onStatusTimer);
  ui_timer_->start();
}

HudPanel::~HudPanel() {
  if (ui_timer_) {
    ui_timer_->stop();
    delete ui_timer_;
    ui_timer_ = nullptr;
  }
}

void HudPanel::onSpeed(const std_msgs::msg::Float64::SharedPtr msg) {
  speed_ = msg->data;
}

void HudPanel::onAction(const std_msgs::msg::Float64::SharedPtr msg) {
  action_id_ = static_cast<int>(msg->data);
}

void HudPanel::onDistance(const std_msgs::msg::Float64::SharedPtr msg) {
  distance_ = msg->data;
}

void HudPanel::onStatusTimer() {
  if (speed_label_) {
    speed_label_->setText(QString("速度: %1 m/s").arg(speed_, 0, 'f', 1));
  }
  if (action_label_) {
    const char* name = "未知";
    if (action_id_ >= 0 && action_id_ <= 3) name = kActionNames[action_id_];
    action_label_->setText(QString("行为: %1").arg(name));
  }
  if (distance_label_) {
    distance_label_->setText(QString("前方距离: %1 m").arg(distance_, 0, 'f', 1));
  }
  // 暂停时按钮显示"继续"
  if (pause_button_) {
    pause_button_->setText(paused_ ? "继续" : "暂停");
  }
}

void HudPanel::onTogglePause() {
  paused_ = !paused_;
  auto msg = std_msgs::msg::Bool();
  msg.data = paused_;
  pause_pub_->publish(msg);
  onStatusTimer();
}

void HudPanel::onClearTrail() {
  auto msg = std_msgs::msg::Bool();
  msg.data = true;
  clear_pub_->publish(msg);
}

void HudPanel::load(const rviz_common::Config& config) {
  Panel::load(config);
}

void HudPanel::save(rviz_common::Config config) const {
  Panel::save(config);
}

}  // namespace sdc

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(sdc::HudPanel, rviz_common::Panel)
