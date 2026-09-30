#pragma once

// ROS-free widget layout, shared by the RViz panel and its offscreen preview.
#include <QFrame>
#include <QGridLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QPushButton>
#include <QScrollArea>
#include <QStyle>
#include <QVBoxLayout>
#include <QVariant>

namespace sdc::hud {
struct Dashboard {
  QLabel *speed, *action, *distance, *mode, *connection, *mission, *position;
  QPushButton *start, *pause, *clear, *automatic, *manual, *reset;
  QFrame* keyboard;
  QLabel *key_w, *key_a, *key_s, *key_d;
};

inline void setTone(QWidget* widget, const char* tone) {
  if (widget->property("tone").toString() == tone) return;
  widget->setProperty("tone", tone);
  widget->style()->unpolish(widget);
  widget->style()->polish(widget);
  widget->update();
}

inline Dashboard buildDashboard(QWidget* panel) {
  Dashboard ui{};
  panel->setObjectName("autonomyHud");
  panel->setMinimumWidth(240);
  panel->setStyleSheet(R"(
    QWidget#autonomyHud { background: #101722; }
    QWidget#autonomyHud QWidget { color: #dbe5f1; font-family: "DejaVu Sans", sans-serif; font-size: 12px; }
    QWidget#autonomyHud QLabel { background: transparent; border: none; }
    QWidget#autonomyHud QFrame[card="true"] { background: #192331; border: 1px solid #2a3748; border-radius: 12px; }
    QWidget#autonomyHud QLabel[role="caption"] { color: #8193aa; font-size: 10px; font-weight: 600; }
    QWidget#autonomyHud QLabel#consoleTitle { color: #f1f5fb; font-size: 19px; font-weight: 600; }
    QWidget#autonomyHud QLabel#speedValue { color: #f1f5fb; font-size: 42px; font-weight: 600; }
    QWidget#autonomyHud QLabel[role="metric"] { color: #e9f0fa; font-size: 21px; font-weight: 600; }
    QWidget#autonomyHud QLabel[role="badge"] { background: #24364b; color: #81cfff; border-radius: 7px; padding: 4px 8px; font-size: 10px; font-weight: 600; }
    QWidget#autonomyHud QLabel[tone="good"] { color: #65dbb0; }
    QWidget#autonomyHud QLabel[tone="warning"] { color: #f7c76d; }
    QWidget#autonomyHud QLabel[tone="danger"] { color: #f59caa; }
    QWidget#autonomyHud QLabel[tone="muted"] { color: #8193aa; }
    QWidget#autonomyHud QPushButton { background: #202d3e; border: 1px solid #344459; border-radius: 8px; padding: 8px 8px; min-height: 20px; font-size: 12px; font-weight: 600; }
    QWidget#autonomyHud QPushButton:hover { background: #2a3b51; border-color: #53708f; }
    QWidget#autonomyHud QPushButton:pressed { background: #152637; }
    QWidget#autonomyHud QPushButton:disabled { color: #6e7d91; background: #182230; border-color: #263244; }
    QWidget#autonomyHud QPushButton#startControl { background: #65cff2; color: #102331; border-color: #65cff2; }
    QWidget#autonomyHud QPushButton#startControl:hover { background: #94dff8; }
    QWidget#autonomyHud QPushButton#startControl:disabled { background: #284654; color: #6e8a98; border-color: #284654; }
    QWidget#autonomyHud QPushButton#resetControl { color: #eba2af; border-color: #62434e; }
    QWidget#autonomyHud QPushButton[role="mode"] { background: transparent; border-color: transparent; color: #8193aa; padding: 7px 3px; }
    QWidget#autonomyHud QPushButton[role="mode"]:checked { background: #304c69; color: #bde9ff; border-color: #426788; }
    QWidget#autonomyHud QFrame#modeSwitch { background: #172230; border: 1px solid #2a3748; border-radius: 10px; }
    QWidget#autonomyHud QLabel[role="key"] { background: #243246; border: 1px solid #3a4d65; border-radius: 6px; min-width: 30px; min-height: 28px; font-weight: 600; }
    QWidget#autonomyHud QLabel[role="key"][tone="pressed"] { background: #65cff2; color: #102331; border-color: #65cff2; }
    QWidget#autonomyHud QScrollArea, QWidget#autonomyHud QWidget#hudContent { background: #101722; border: none; }
    QWidget#autonomyHud QScrollBar:vertical { background: #101722; width: 7px; margin: 0; }
    QWidget#autonomyHud QScrollBar::handle:vertical { background: #3a4d65; border-radius: 3px; min-height: 24px; }
    QWidget#autonomyHud QScrollBar::add-line:vertical, QWidget#autonomyHud QScrollBar::sub-line:vertical { height: 0; }
    QWidget#autonomyHud QScrollBar::add-page:vertical, QWidget#autonomyHud QScrollBar::sub-page:vertical { background: none; }
  )");
  auto* outer = new QVBoxLayout(panel);
  outer->setContentsMargins(10, 10, 10, 10);
  auto* scroll = new QScrollArea(panel);
  scroll->setFrameShape(QFrame::NoFrame);
  scroll->setWidgetResizable(true);
  scroll->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
  auto* content = new QWidget(scroll);
  content->setObjectName("hudContent");
  auto* root = new QVBoxLayout(content);
  root->setContentsMargins(0, 0, 1, 0);
  root->setSpacing(10);
  scroll->setWidget(content);
  outer->addWidget(scroll);

  auto label = [](const QString& text, QWidget* parent, const char* role = "") {
    auto* widget = new QLabel(text, parent);
    widget->setProperty("role", role);
    return widget;
  };
  auto card = [content]() {
    auto* frame = new QFrame(content);
    frame->setProperty("card", true);
    frame->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Maximum);
    return frame;
  };
  auto button = [content](const QString& text, const QString& tooltip) {
    auto* widget = new QPushButton(text, content);
    widget->setToolTip(tooltip);
    widget->setFocusPolicy(Qt::NoFocus); // Keep WASD focus on the panel.
    widget->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
    return widget;
  };

  auto* header = new QVBoxLayout();
  header->setSpacing(4);
  auto* title = label("Drive console", content);
  title->setObjectName("consoleTitle");
  header->addWidget(title);
  ui.connection = label("WAITING FOR TELEMETRY", content, "caption");
  header->addWidget(ui.connection);
  root->addLayout(header);

  auto* speed_card = card();
  auto* speed_layout = new QVBoxLayout(speed_card);
  speed_layout->setContentsMargins(14, 12, 14, 12);
  speed_layout->setSpacing(7);
  auto* speed_header = new QHBoxLayout();
  speed_header->addWidget(label("VEHICLE SPEED", speed_card, "caption"));
  speed_header->addStretch();
  ui.mode = label("AUTO", speed_card, "badge");
  speed_header->addWidget(ui.mode);
  speed_layout->addLayout(speed_header);
  auto* speed_row = new QHBoxLayout();
  ui.speed = label("--", speed_card);
  ui.speed->setObjectName("speedValue");
  speed_row->addWidget(ui.speed);
  speed_row->addWidget(label("m/s", speed_card, "caption"), 0, Qt::AlignBottom);
  speed_row->addStretch();
  speed_layout->addLayout(speed_row);
  auto* mission_row = new QHBoxLayout();
  mission_row->addWidget(label("MISSION", speed_card, "caption"));
  ui.mission = label("--", speed_card);
  ui.mission->setWordWrap(true);
  ui.mission->setAlignment(Qt::AlignRight);
  mission_row->addWidget(ui.mission, 1);
  speed_layout->addLayout(mission_row);
  root->addWidget(speed_card);

  auto* metrics = new QHBoxLayout();
  metrics->setSpacing(8);
  auto metric = [&](const QString& caption) {
    auto* frame = card();
    auto* layout = new QVBoxLayout(frame);
    layout->setContentsMargins(12, 10, 12, 10);
    layout->setSpacing(5);
    layout->addWidget(label(caption, frame, "caption"));
    auto* value = label("--", frame, "metric");
    value->setMinimumWidth(0);
    layout->addWidget(value);
    metrics->addWidget(frame, 1);
    return value;
  };
  ui.distance = metric("FRONT / m");
  ui.action = metric("ACTION");
  root->addLayout(metrics);

  auto* position_card = card();
  auto* position_layout = new QVBoxLayout(position_card);
  position_layout->setContentsMargins(12, 10, 12, 10);
  position_layout->setSpacing(5);
  position_layout->addWidget(label("POSITION / m", position_card, "caption"));
  ui.position = label("X --     Y --     Z --", position_card);
  ui.position->setWordWrap(true);
  position_layout->addWidget(ui.position);
  root->addWidget(position_card);

  root->addWidget(label("DRIVING MODE", content, "caption"));
  auto* mode_switch = new QFrame(content);
  mode_switch->setObjectName("modeSwitch");
  auto* mode_row = new QHBoxLayout(mode_switch);
  mode_row->setContentsMargins(4, 4, 4, 4);
  mode_row->setSpacing(4);
  ui.automatic = button("AUTO", "Follow the planned trajectory");
  ui.manual = button("MANUAL", "Drive with W / A / S / D");
  ui.automatic->setObjectName("automaticMode");
  ui.manual->setObjectName("manualMode");
  for (auto* control : {ui.automatic, ui.manual}) {
    control->setProperty("role", "mode");
    control->setCheckable(true);
    mode_row->addWidget(control, 1);
  }
  ui.automatic->setChecked(true);
  root->addWidget(mode_switch);

  root->addWidget(label("DRIVE CONTROLS", content, "caption"));
  auto* controls = new QGridLayout();
  controls->setSpacing(8);
  ui.start = button("Start", "Start or resume driving");
  ui.start->setObjectName("startControl");
  ui.pause = button("Pause", "Pause or resume the current run");
  ui.clear = button("Clear trail", "Remove the displayed vehicle trail");
  ui.reset = button("Reset car", "Return the vehicle to the scenario start");
  ui.reset->setObjectName("resetControl");
  controls->addWidget(ui.start, 0, 0);
  controls->addWidget(ui.pause, 0, 1);
  controls->addWidget(ui.clear, 1, 0);
  controls->addWidget(ui.reset, 1, 1);
  controls->setColumnStretch(0, 1);
  controls->setColumnStretch(1, 1);
  root->addLayout(controls);

  ui.keyboard = card();
  auto* keyboard_layout = new QVBoxLayout(ui.keyboard);
  keyboard_layout->setContentsMargins(12, 10, 12, 10);
  keyboard_layout->addWidget(label("MANUAL DRIVE", ui.keyboard, "caption"));
  auto* keys = new QGridLayout();
  keys->setSpacing(5);
  auto key = [&](const QString& text, int row, int column) {
    auto* widget = label(text, ui.keyboard, "key");
    widget->setAlignment(Qt::AlignCenter);
    keys->addWidget(widget, row, column);
    return widget;
  };
  ui.key_w = key("W", 0, 1);
  ui.key_a = key("A", 1, 0);
  ui.key_s = key("S", 1, 1);
  ui.key_d = key("D", 1, 2);
  keyboard_layout->addLayout(keys);
  auto* hint = label("Click the panel to focus.\nW / S move  ·  A / D steer", ui.keyboard);
  hint->setWordWrap(true);
  setTone(hint, "muted");
  keyboard_layout->addWidget(hint);
  root->addWidget(ui.keyboard);
  ui.keyboard->hide();
  root->addStretch(1);
  return ui;
}
}  // namespace sdc::hud
