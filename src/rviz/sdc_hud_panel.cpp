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
    , exam_label_(nullptr)
    , map_combo_(nullptr)
    , pause_button_(nullptr)
    , clear_button_(nullptr)
    , exam_button_(nullptr)
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
  state_layout->addWidget(speed_label_);
  state_layout->addWidget(action_label_);
  state_layout->addWidget(distance_label_);
  root->addWidget(state_box);

  // 地图选择区
  auto* map_box = new QGroupBox("地图 / 场景", this);
  auto* map_layout = new QVBoxLayout(map_box);
  map_combo_ = new QComboBox(map_box);
  map_combo_->addItem("环形道路");
  map_combo_->addItem("倒车入库");
  map_combo_->addItem("侧方停车");
  map_combo_->addItem("直角转弯");
  map_layout->addWidget(map_combo_);
  root->addWidget(map_box);

  // 考试区
  auto* exam_box = new QGroupBox("科目二考试", this);
  auto* exam_layout = new QVBoxLayout(exam_box);
  exam_label_ = new QLabel("考试: 待开始", exam_box);
  exam_button_ = new QPushButton("开始考试", exam_box);
  exam_layout->addWidget(exam_label_);
  exam_layout->addWidget(exam_button_);
  root->addWidget(exam_box);

  // 控制区
  auto* ctrl_box = new QGroupBox("控制", this);
  auto* ctrl_layout = new QHBoxLayout(ctrl_box);
  pause_button_ = new QPushButton("暂停", ctrl_box);
  clear_button_ = new QPushButton("清除轨迹", ctrl_box);
  reset_button_ = new QPushButton("重置小车", ctrl_box);
  ctrl_layout->addWidget(pause_button_);
  ctrl_layout->addWidget(clear_button_);
  ctrl_layout->addWidget(reset_button_);
  root->addWidget(ctrl_box);

  // 信号连接
  connect(pause_button_, &QPushButton::clicked, this, &HudPanel::onTogglePause);
  connect(clear_button_, &QPushButton::clicked, this, &HudPanel::onClearTrail);
  connect(reset_button_, &QPushButton::clicked, this, &HudPanel::onResetCar);
  connect(exam_button_, &QPushButton::clicked, this, &HudPanel::onStartExam);
  connect(map_combo_, QOverload<int>::of(&QComboBox::currentIndexChanged),
          this, &HudPanel::onMapChanged);

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
  exam_status_sub_ = node_->create_subscription<std_msgs::msg::String>(
      "sdc/exam_status", qos, std::bind(&HudPanel::onExamStatus, this, std::placeholders::_1));
  exam_progress_sub_ = node_->create_subscription<std_msgs::msg::String>(
      "sdc/exam_progress", qos, std::bind(&HudPanel::onExamProgress, this, std::placeholders::_1));

  pause_pub_ = node_->create_publisher<std_msgs::msg::Bool>("sdc/pause", qos);
  clear_pub_ = node_->create_publisher<std_msgs::msg::Bool>("sdc/clear_trail", qos);
  start_exam_pub_ = node_->create_publisher<std_msgs::msg::Bool>("sdc/start_exam", qos);
  reset_pub_ = node_->create_publisher<std_msgs::msg::Bool>("sdc/reset_car", qos);
  set_map_pub_ = node_->create_publisher<std_msgs::msg::Int32>("sdc/set_map", qos);

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

void HudPanel::onSpeed(const std_msgs::msg::Float64::SharedPtr msg) { speed_ = msg->data; }
void HudPanel::onAction(const std_msgs::msg::Float64::SharedPtr msg) { action_id_ = static_cast<int>(msg->data); }
void HudPanel::onDistance(const std_msgs::msg::Float64::SharedPtr msg) { distance_ = msg->data; }
void HudPanel::onExamStatus(const std_msgs::msg::String::SharedPtr msg) { exam_status_ = msg->data; }
void HudPanel::onExamProgress(const std_msgs::msg::String::SharedPtr msg) { exam_progress_ = msg->data; }

void HudPanel::onStatusTimer() {
  if (speed_label_)  speed_label_->setText(QString("速度: %1 m/s").arg(speed_, 0, 'f', 1));
  if (action_label_) {
    const char* name = "未知";
    if (action_id_ >= 0 && action_id_ <= 3) name = kActionNames[action_id_];
    action_label_->setText(QString("行为: %1").arg(name));
  }
  if (distance_label_) distance_label_->setText(QString("前方距离: %1 m").arg(distance_, 0, 'f', 1));
  if (exam_label_) {
    QString t = QString::fromStdString(exam_status_);
    if (!exam_progress_.empty())
      t += QString("  (%1)").arg(QString::fromStdString(exam_progress_));
    exam_label_->setText("考试: " + t);
  }
  if (pause_button_) pause_button_->setText(paused_ ? "继续" : "暂停");
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

void HudPanel::onResetCar() {
  auto msg = std_msgs::msg::Bool(); msg.data = true;
  reset_pub_->publish(msg);
}

void HudPanel::onStartExam() {
  auto msg = std_msgs::msg::Bool(); msg.data = true;
  start_exam_pub_->publish(msg);
}

void HudPanel::onMapChanged(int index) {
  auto msg = std_msgs::msg::Int32(); msg.data = index;  // 0=环形 1=倒车入库 ...
  set_map_pub_->publish(msg);
  // 切地图会自动重置考试，按钮文案复位
  exam_status_.clear();
  exam_progress_.clear();
}

void HudPanel::load(const rviz_common::Config& config) { Panel::load(config); }
void HudPanel::save(rviz_common::Config config) const { Panel::save(config); }

}  // namespace sdc

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(sdc::HudPanel, rviz_common::Panel)
