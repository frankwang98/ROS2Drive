/**
 * sdc_hud_panel.cpp — 自动驾驶小车 HUD 控制面板实现
 *
 * 功能：
 *   - 自动 / 手动驾驶模式切换
 *   - 手动模式下 WASD 键盘控制（W=前进 S=倒车 A=左转 D=右转）
 *   - 开始 / 暂停、清除轨迹、重置小车
 */

#include "sdc_hud_panel.hpp"

#include <cmath>
#include <QMetaObject>

#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QGridLayout>
#include <QString>

#include <rviz_common/display_context.hpp>

namespace sdc {

static const char* kActionNames[] = {"ACCEL", "CRUISE", "BRAKE", "STOP"};

HudPanel::HudPanel(QWidget* parent) : rviz_common::Panel(parent) {
  dashboard_ = hud::buildDashboard(this);
  speed_label_ = dashboard_.speed;
  action_label_ = dashboard_.action;
  distance_label_ = dashboard_.distance;
  mode_label_ = dashboard_.mode;
  start_button_ = dashboard_.start;
  pause_button_ = dashboard_.pause;
  clear_button_ = dashboard_.clear;
  mode_button_ = dashboard_.manual;
  reset_button_ = dashboard_.reset;

  connect(start_button_, &QPushButton::clicked, this, &HudPanel::onStart);
  connect(pause_button_, &QPushButton::clicked, this, &HudPanel::onTogglePause);
  connect(clear_button_, &QPushButton::clicked, this, &HudPanel::onClearTrail);
  connect(mode_button_, &QPushButton::clicked, this, [this] {
    if (!manual_) onToggleMode();
    else onStatusTimer();
    setFocus(Qt::OtherFocusReason);
  });
  connect(dashboard_.automatic, &QPushButton::clicked, this, [this] {
    if (manual_) onToggleMode();
    else onStatusTimer();
    setFocus(Qt::OtherFocusReason);
  });
  connect(reset_button_, &QPushButton::clicked, this, &HudPanel::onResetCar);
  for (auto* button : {start_button_, pause_button_, clear_button_, mode_button_,
                       dashboard_.automatic, reset_button_})
    button->setEnabled(false);
  setFocusPolicy(Qt::StrongFocus);
}

void HudPanel::onInitialize() {
  if (!getDisplayContext())
    return;
  auto ros_node_abs = getDisplayContext()->getRosNodeAbstraction().lock();
  if (!ros_node_abs)
    return;
  initializeRos(ros_node_abs->get_raw_node());
}

void HudPanel::initializeRos(rclcpp::Node::SharedPtr node) {
  if (!node || node_) return;
  node_ = std::move(node);

  auto qos = rclcpp::QoS(10);
  speed_sub_ = node_->create_subscription<std_msgs::msg::Float64>(
      "sdc/speed", qos, std::bind(&HudPanel::onSpeed, this, std::placeholders::_1));
  action_sub_ = node_->create_subscription<std_msgs::msg::Float64>(
      "sdc/action_id", qos, std::bind(&HudPanel::onAction, this, std::placeholders::_1));
  distance_sub_ = node_->create_subscription<std_msgs::msg::Float64>(
      "sdc/front_distance", qos, std::bind(&HudPanel::onDistance, this, std::placeholders::_1));
  auto odom_qos = rclcpp::SensorDataQoS().keep_last(5);
  odom_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(
      "sdc/odometry", odom_qos, std::bind(&HudPanel::onOdometry, this, std::placeholders::_1));
  mode_sub_ = node_->create_subscription<std_msgs::msg::Int32>(
      "sdc/mode", qos, std::bind(&HudPanel::onMode, this, std::placeholders::_1));
  mission_stage_sub_ = node_->create_subscription<std_msgs::msg::String>(
      "sdc/mission_stage", qos, std::bind(&HudPanel::onMissionStage, this, std::placeholders::_1));

  start_pub_ = node_->create_publisher<std_msgs::msg::Bool>("sdc/start", qos);
  pause_pub_ = node_->create_publisher<std_msgs::msg::Bool>("sdc/pause", qos);
  clear_pub_ = node_->create_publisher<std_msgs::msg::Bool>("sdc/clear_trail", qos);
  set_mode_pub_ = node_->create_publisher<std_msgs::msg::Int32>("sdc/set_mode", qos);
  reset_pub_ = node_->create_publisher<std_msgs::msg::Bool>("sdc/reset_car", qos);
  manual_pub_ = node_->create_publisher<geometry_msgs::msg::Twist>("sdc/manual_cmd", qos);

  for (auto* button : {start_button_, pause_button_, clear_button_, mode_button_,
                       dashboard_.automatic, reset_button_})
    button->setEnabled(true);
  ui_timer_ = new QTimer(this);
  ui_timer_->setInterval(50);  // 20Hz 手动指令发送
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

// ROS subscriptions may run outside the Qt GUI thread. Queue state updates
// through the panel's QObject context; queued work is cancelled on destruction.
void HudPanel::onSpeed(const std_msgs::msg::Float64::SharedPtr msg) {
  QMetaObject::invokeMethod(this, [this, value = msg->data] {
    speed_ = value;
    have_speed_ = std::isfinite(value);
    telemetry_clock_.restart();
  }, Qt::QueuedConnection);
}
void HudPanel::onAction(const std_msgs::msg::Float64::SharedPtr msg) {
  QMetaObject::invokeMethod(this, [this, value = msg->data] {
    action_id_ = std::isfinite(value) && value >= 0.0 && value <= 3.0
                     ? static_cast<int>(value) : -1;
  }, Qt::QueuedConnection);
}
void HudPanel::onDistance(const std_msgs::msg::Float64::SharedPtr msg) {
  QMetaObject::invokeMethod(this, [this, value = msg->data] {
    distance_ = value;
    have_distance_ = std::isfinite(value);
  }, Qt::QueuedConnection);
}
void HudPanel::onOdometry(const nav_msgs::msg::Odometry::SharedPtr msg) {
  const auto position = msg->pose.pose.position;
  QMetaObject::invokeMethod(this, [this, position] {
    pose_x_ = position.x;
    pose_y_ = position.y;
    pose_z_ = position.z;
    have_position_ = std::isfinite(pose_x_) && std::isfinite(pose_y_) && std::isfinite(pose_z_);
  }, Qt::QueuedConnection);
}
void HudPanel::onMode(const std_msgs::msg::Int32::SharedPtr msg) {
  QMetaObject::invokeMethod(this, [this, value = msg->data] {
    manual_ = value == 1;
    if (!manual_) {
      key_w_ = key_s_ = key_a_ = key_d_ = false;
      publishManualCmd();
    }
    onStatusTimer();
  }, Qt::QueuedConnection);
}
void HudPanel::onMissionStage(const std_msgs::msg::String::SharedPtr msg) {
  QMetaObject::invokeMethod(this, [this, value = msg->data] {
    mission_stage_ = value.empty() ? "--" : value;
  }, Qt::QueuedConnection);
}

void HudPanel::onStatusTimer() {
  speed_label_->setText(have_speed_ ? QString::number(speed_, 'f', 1) : "--");
  const char* action = "--";
  if (action_id_ >= 0 && action_id_ <= 3) action = kActionNames[action_id_];
  action_label_->setText(action);
  hud::setTone(action_label_, action_id_ >= 2 ? "warning" : "good");
  distance_label_->setText(have_distance_ ? QString::number(distance_, 'f', 1) : "--");
  hud::setTone(distance_label_, have_distance_ && distance_ < 3.0 ? "warning" : "normal");
  mode_label_->setText(manual_ ? "MANUAL" : "AUTO");
  dashboard_.automatic->setChecked(!manual_);
  mode_button_->setChecked(manual_);
  dashboard_.keyboard->setVisible(manual_);
  dashboard_.mission->setText(QString::fromStdString(mission_stage_));
  dashboard_.position->setText(have_position_
      ? QString("X %1   Y %2   Z %3").arg(pose_x_, 0, 'f', 1).arg(pose_y_, 0, 'f', 1).arg(pose_z_, 0, 'f', 1)
      : "X --   Y --   Z --");
  const bool live = telemetry_clock_.isValid() && telemetry_clock_.elapsed() < 2000;
  dashboard_.connection->setText(live ? (paused_ ? "LIVE TELEMETRY  /  PAUSED" : "LIVE TELEMETRY")
      : telemetry_clock_.isValid() ? "TELEMETRY STALE" : "WAITING FOR TELEMETRY");
  hud::setTone(dashboard_.connection, live ? "good" : "muted");
  pause_button_->setText(paused_ ? "Resume" : "Pause");
  hud::setTone(dashboard_.key_w, key_w_ ? "pressed" : "normal");
  hud::setTone(dashboard_.key_a, key_a_ ? "pressed" : "normal");
  hud::setTone(dashboard_.key_s, key_s_ ? "pressed" : "normal");
  hud::setTone(dashboard_.key_d, key_d_ ? "pressed" : "normal");
  if (manual_) publishManualCmd();
}

// ---- 键盘事件（WASD）----
void HudPanel::keyPressEvent(QKeyEvent* event) {
  const int key = event->key();
  if (manual_ && (key == Qt::Key_W || key == Qt::Key_A || key == Qt::Key_S || key == Qt::Key_D)) {
    if (!event->isAutoRepeat()) updateKey(key, true);
    event->accept();
    return;
  }
  QWidget::keyPressEvent(event);
}

void HudPanel::keyReleaseEvent(QKeyEvent* event) {
  const int key = event->key();
  if (manual_ && (key == Qt::Key_W || key == Qt::Key_A || key == Qt::Key_S || key == Qt::Key_D)) {
    if (!event->isAutoRepeat()) updateKey(key, false);
    event->accept();
    return;
  }
  QWidget::keyReleaseEvent(event);
}

void HudPanel::focusOutEvent(QFocusEvent* event) {
  // Losing panel focus must never leave a held throttle/steering command.
  key_w_ = key_s_ = key_a_ = key_d_ = false;
  if (manual_) publishManualCmd();
  QWidget::focusOutEvent(event);
}

void HudPanel::updateKey(int key, bool pressed) {
  bool changed = false;
  switch (key) {
    case Qt::Key_W:
      if (key_w_ != pressed) {
        key_w_ = pressed;
        changed = true;
      }
      break;
    case Qt::Key_S:
      if (key_s_ != pressed) {
        key_s_ = pressed;
        changed = true;
      }
      break;
    case Qt::Key_A:
      if (key_a_ != pressed) {
        key_a_ = pressed;
        changed = true;
      }
      break;
    case Qt::Key_D:
      if (key_d_ != pressed) {
        key_d_ = pressed;
        changed = true;
      }
      break;
    default:
      break;
  }
  if (changed)
    publishManualCmd();
}

void HudPanel::publishManualCmd() {
  if (!manual_pub_)
    return;
  geometry_msgs::msg::Twist twist;
  // 油门：W 前进(+1)，S 倒车(-1)；同时按则相互抵消
  double throttle = 0.0;
  if (key_w_)
    throttle += 1.0;
  if (key_s_)
    throttle -= 1.0;
  // 转向：A 左(+1)，D 右(-1)
  // （阿克曼模型 steer>0 => yaw 增大 => 左转，故左=正转角，右=负转角）
  double steer = 0.0;
  if (key_a_)
    steer += 1.0;
  if (key_d_)
    steer -= 1.0;
  twist.linear.x = throttle;
  twist.angular.z = steer;
  manual_pub_->publish(twist);
}

// ---- 控制按钮 ----
void HudPanel::onStart() {
  if (!start_pub_) return;
  auto msg = std_msgs::msg::Bool();
  msg.data = true;
  start_pub_->publish(msg);
  paused_ = false;
  onStatusTimer();
}

void HudPanel::onTogglePause() {
  if (!pause_pub_) return;
  paused_ = !paused_;
  auto msg = std_msgs::msg::Bool();
  msg.data = paused_;
  pause_pub_->publish(msg);
  onStatusTimer();
}

void HudPanel::onClearTrail() {
  if (!clear_pub_) return;
  auto msg = std_msgs::msg::Bool();
  msg.data = true;
  clear_pub_->publish(msg);
}

void HudPanel::onToggleMode() {
  if (!set_mode_pub_) return;
  manual_ = !manual_;
  auto msg = std_msgs::msg::Int32();
  msg.data = manual_ ? 1 : 0;
  set_mode_pub_->publish(msg);
  if (!manual_) {
    key_w_ = key_s_ = key_a_ = key_d_ = false;
    publishManualCmd();
  }
  onStatusTimer();
}

void HudPanel::onResetCar() {
  if (!reset_pub_) return;
  key_w_ = key_s_ = key_a_ = key_d_ = false;
  if (manual_) publishManualCmd();
  auto msg = std_msgs::msg::Bool();
  msg.data = true;
  reset_pub_->publish(msg);
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

