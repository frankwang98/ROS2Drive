/**
 * sdc_hud_panel.cpp — 自动驾驶小车 HUD 控制面板实现
 *
 * 功能：
 *   - 自动 / 手动驾驶模式切换
 *   - 手动模式下 WASD 键盘控制（W=前进 S=倒车 A=左转 D=右转）
 *   - 开始 / 暂停、清除轨迹、重置小车
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
    , mode_label_(nullptr)
    , start_button_(nullptr)
    , pause_button_(nullptr)
    , clear_button_(nullptr)
    , mode_button_(nullptr)
    , reset_button_(nullptr)
    , ui_timer_(nullptr)
{
  auto* root = new QVBoxLayout(this);

  // 状态区
  auto* state_box = new QGroupBox("小车状态", this);
  auto* state_layout = new QVBoxLayout(state_box);
  speed_label_ = new QLabel("速度: -- m/s", state_box);
  action_label_ = new QLabel("行为: --", state_box);
  distance_label_ = new QLabel("前方距离: -- m", state_box);
  mode_label_ = new QLabel("驾驶模式: 自动", state_box);
  state_layout->addWidget(speed_label_);
  state_layout->addWidget(action_label_);
  state_layout->addWidget(distance_label_);
  state_layout->addWidget(mode_label_);
  root->addWidget(state_box);

  // 驾驶模式区
  auto* mode_box = new QGroupBox("驾驶模式", this);
  auto* mode_layout = new QVBoxLayout(mode_box);
  mode_button_ = new QPushButton("切换到手动 (WASD)", mode_box);
  mode_layout->addWidget(mode_button_);
  auto* hint = new QLabel("手动模式：W=前进  S=倒车  A=左转  D=右转", mode_box);
  hint->setWordWrap(true);
  mode_layout->addWidget(hint);
  root->addWidget(mode_box);

  // 行驶控制区
  auto* ctrl_box = new QGroupBox("行驶控制", this);
  auto* ctrl_layout = new QVBoxLayout(ctrl_box);
  auto* ctrl_row = new QHBoxLayout();
  start_button_ = new QPushButton("开始", ctrl_box);
  pause_button_ = new QPushButton("暂停", ctrl_box);
  clear_button_ = new QPushButton("清除轨迹", ctrl_box);
  ctrl_row->addWidget(start_button_);
  ctrl_row->addWidget(pause_button_);
  ctrl_row->addWidget(clear_button_);
  ctrl_layout->addLayout(ctrl_row);
  reset_button_ = new QPushButton("重置小车", ctrl_box);
  ctrl_layout->addWidget(reset_button_);
  root->addWidget(ctrl_box);

  // 信号连接
  connect(start_button_, &QPushButton::clicked, this, &HudPanel::onStart);
  connect(pause_button_, &QPushButton::clicked, this, &HudPanel::onTogglePause);
  connect(clear_button_, &QPushButton::clicked, this, &HudPanel::onClearTrail);
  connect(mode_button_, &QPushButton::clicked, this, &HudPanel::onToggleMode);
  connect(reset_button_, &QPushButton::clicked, this, &HudPanel::onResetCar);

  // 允许面板接收键盘焦点（WASD 控制需要）
  setFocusPolicy(Qt::StrongFocus);
  setFocus();

  // 速度大字体
  QFont font = speed_label_->font();
  font.setPointSize(14);
  font.setBold(true);
  speed_label_->setFont(font);
}

void HudPanel::onInitialize() {
  if (!getDisplayContext()) return;
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
  mode_sub_ = node_->create_subscription<std_msgs::msg::Int32>(
      "sdc/mode", qos, std::bind(&HudPanel::onMode, this, std::placeholders::_1));

  start_pub_ = node_->create_publisher<std_msgs::msg::Bool>("sdc/start", qos);
  pause_pub_ = node_->create_publisher<std_msgs::msg::Bool>("sdc/pause", qos);
  clear_pub_ = node_->create_publisher<std_msgs::msg::Bool>("sdc/clear_trail", qos);
  set_mode_pub_ = node_->create_publisher<std_msgs::msg::Int32>("sdc/set_mode", qos);
  reset_pub_ = node_->create_publisher<std_msgs::msg::Bool>("sdc/reset_car", qos);
  manual_pub_ = node_->create_publisher<geometry_msgs::msg::Twist>("sdc/manual_cmd", qos);

  ui_timer_ = new QTimer(this);
  ui_timer_->setInterval(50);   // 20Hz 手动指令发送
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

void HudPanel::onSpeed(const std_msgs::msg::Float64::SharedPtr msg) { speed_ = msg->data; }
void HudPanel::onAction(const std_msgs::msg::Float64::SharedPtr msg) { action_id_ = static_cast<int>(msg->data); }
void HudPanel::onDistance(const std_msgs::msg::Float64::SharedPtr msg) { distance_ = msg->data; }
void HudPanel::onMode(const std_msgs::msg::Int32::SharedPtr msg) {
  manual_ = (msg->data == 1);
  mode_label_->setText(QString("驾驶模式: %1").arg(manual_ ? "手动 (WASD)" : "自动"));
  mode_button_->setText(manual_ ? "切换到自动" : "切换到手动 (WASD)");
  if (!manual_) {
    // 切回自动时松开全部键
    key_w_ = key_s_ = key_a_ = key_d_ = false;
    publishManualCmd();
  }
}

void HudPanel::onStatusTimer() {
  if (speed_label_)  speed_label_->setText(QString("速度: %1 m/s").arg(speed_, 0, 'f', 1));
  if (action_label_) {
    const char* name = "未知";
    if (action_id_ >= 0 && action_id_ <= 3) name = kActionNames[action_id_];
    action_label_->setText(QString("行为: %1").arg(name));
  }
  if (distance_label_) distance_label_->setText(QString("前方距离: %1 m").arg(distance_, 0, 'f', 1));
  if (pause_button_) pause_button_->setText(paused_ ? "继续" : "暂停");

  // 手动模式下持续发送指令（保证松开后自动回中）
  if (manual_) publishManualCmd();
}

// ---- 键盘事件（WASD）----
void HudPanel::keyPressEvent(QKeyEvent* event) {
  if (manual_) updateKey(event->key(), true);
  QWidget::keyPressEvent(event);
}

void HudPanel::keyReleaseEvent(QKeyEvent* event) {
  if (manual_) updateKey(event->key(), false);
  QWidget::keyReleaseEvent(event);
}

void HudPanel::updateKey(int key, bool pressed) {
  bool changed = false;
  switch (key) {
    case Qt::Key_W: if (key_w_ != pressed) { key_w_ = pressed; changed = true; } break;
    case Qt::Key_S: if (key_s_ != pressed) { key_s_ = pressed; changed = true; } break;
    case Qt::Key_A: if (key_a_ != pressed) { key_a_ = pressed; changed = true; } break;
    case Qt::Key_D: if (key_d_ != pressed) { key_d_ = pressed; changed = true; } break;
    default: break;
  }
  if (changed) publishManualCmd();
}

void HudPanel::publishManualCmd() {
  if (!manual_pub_) return;
  geometry_msgs::msg::Twist twist;
  // 油门：W 前进(+1)，S 倒车(-1)；同时按则相互抵消
  double throttle = 0.0;
  if (key_w_) throttle += 1.0;
  if (key_s_) throttle -= 1.0;
  // 转向：A 左(-1)，D 右(+1)
  double steer = 0.0;
  if (key_a_) steer -= 1.0;
  if (key_d_) steer += 1.0;
  twist.linear.x = throttle;
  twist.angular.z = steer;
  manual_pub_->publish(twist);
}

// ---- 控制按钮 ----
void HudPanel::onStart() {
  auto msg = std_msgs::msg::Bool(); msg.data = true;
  start_pub_->publish(msg);
  paused_ = false;
  onStatusTimer();
}

void HudPanel::onTogglePause() {
  paused_ = !paused_;
  auto msg = std_msgs::msg::Bool(); msg.data = paused_;
  pause_pub_->publish(msg);
  onStatusTimer();
}

void HudPanel::onClearTrail() {
  auto msg = std_msgs::msg::Bool(); msg.data = true;
  clear_pub_->publish(msg);
}

void HudPanel::onToggleMode() {
  manual_ = !manual_;
  auto msg = std_msgs::msg::Int32(); msg.data = manual_ ? 1 : 0;
  set_mode_pub_->publish(msg);
  if (!manual_) {
    key_w_ = key_s_ = key_a_ = key_d_ = false;
    publishManualCmd();
  }
  onStatusTimer();
}

void HudPanel::onResetCar() {
  auto msg = std_msgs::msg::Bool(); msg.data = true;
  reset_pub_->publish(msg);
}

void HudPanel::load(const rviz_common::Config& config) { Panel::load(config); }
void HudPanel::save(rviz_common::Config config) const { Panel::save(config); }

}  // namespace sdc

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(sdc::HudPanel, rviz_common::Panel)
