// Actual Qt key events -> HudPanel -> ROS Twist -> simulation vehicle model.
#include "sdc_hud_panel.hpp"
#include "vehicle/simulated_vehicle.hpp"

#include <QApplication>
#include <QDir>
#include <QTest>
#include <cmath>
#include <functional>
#include <iostream>
#include <stdexcept>

class TestPanel : public sdc::HudPanel {
 public:
  using HudPanel::initializeRos;
};

int main(int argc, char** argv) {
  QApplication app(argc, argv);
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("hud_offscreen_check");
  try {
    TestPanel panel;
    panel.initializeRos(node);
    panel.resize(320, 700);
    panel.show();
    panel.activateWindow();
    panel.setFocus();
    geometry_msgs::msg::Twist latest;
    int commands = 0, mode = -1;
    auto twist_sub = node->create_subscription<geometry_msgs::msg::Twist>(
        "sdc/manual_cmd", 10, [&](geometry_msgs::msg::Twist::SharedPtr msg) {
          latest = *msg;
          ++commands;
        });
    auto mode_sub = node->create_subscription<std_msgs::msg::Int32>(
        "sdc/set_mode", 10, [&](std_msgs::msg::Int32::SharedPtr msg) { mode = msg->data; });
    auto speed = node->create_publisher<std_msgs::msg::Float64>("sdc/speed", 10);
    auto action = node->create_publisher<std_msgs::msg::Float64>("sdc/action_id", 10);
    auto distance = node->create_publisher<std_msgs::msg::Float64>("sdc/front_distance", 10);
    auto stage = node->create_publisher<std_msgs::msg::String>("sdc/mission_stage", 10);
    auto odom = node->create_publisher<nav_msgs::msg::Odometry>("sdc/odometry", rclcpp::SensorDataQoS());
    auto pump = [&] { rclcpp::spin_some(node); app.processEvents(); QTest::qWait(10); };
    auto wait = [&](const std::function<bool()>& predicate, const char* message) {
      for (int i = 0; i < 500 && !predicate(); ++i) pump();
      if (!predicate()) throw std::runtime_error(message);
    };
    wait([&] { return twist_sub->get_publisher_count() > 0 && mode_sub->get_publisher_count() > 0 &&
                      speed->get_subscription_count() > 0 && odom->get_subscription_count() > 0; },
         "DDS discovery failed");
    auto* manual = panel.findChild<QPushButton*>("manualMode");
    auto* automatic = panel.findChild<QPushButton*>("automaticMode");
    if (!manual || !automatic) throw std::runtime_error("missing mode controls");
    QTest::mouseClick(manual, Qt::LeftButton);
    wait([&] { return mode == 1 && manual->isChecked(); }, "manual mode request not published");
    if (app.focusWidget() != &panel) throw std::runtime_error("mode button stole keyboard focus");
    auto expect_command = [&](double throttle, double steering, const char* message) {
      const int before = commands;
      wait([&] { return commands > before && latest.linear.x == throttle && latest.angular.z == steering; }, message);
    };
    auto press = [&](Qt::Key key) { QTest::keyPress(app.focusWidget(), key); };
    auto release = [&](Qt::Key key) { QTest::keyRelease(app.focusWidget(), key); };
    press(Qt::Key_W); expect_command(1, 0, "W must publish forward throttle");
    // Synthetic auto-repeat release must not drop a physically held key.
    QKeyEvent repeat(QEvent::KeyRelease, Qt::Key_W, Qt::NoModifier, "w", true);
    QApplication::sendEvent(&panel, &repeat);
    expect_command(1, 0, "auto-repeat release dropped held W");
    press(Qt::Key_A); expect_command(1, 1, "W+A must publish forward/left");
    sdc::vehicle::SimulatedVehicle car;
    car.reset(0, 0, 0);
    for (int i = 0; i < 30; ++i) car.applyManual(latest.linear.x, latest.angular.z, 0.05);
    if (!(car.model().x() > 0 && car.model().yaw() > 0)) throw std::runtime_error("W+A vehicle motion wrong");
    release(Qt::Key_A); expect_command(1, 0, "release A must centre steering");
    press(Qt::Key_D); expect_command(1, -1, "W+D must publish forward/right");
    car.reset(0, 0, 0);
    for (int i = 0; i < 30; ++i) car.applyManual(latest.linear.x, latest.angular.z, 0.05);
    if (!(car.model().x() > 0 && car.model().yaw() < 0)) throw std::runtime_error("W+D vehicle motion wrong");
    release(Qt::Key_D); release(Qt::Key_W); expect_command(0, 0, "release all keys must stop command");
    press(Qt::Key_S); expect_command(-1, 0, "S must publish reverse throttle");
    car.reset(0, 0, 0);
    for (int i = 0; i < 30; ++i) car.applyManual(latest.linear.x, latest.angular.z, 0.05);
    if (!(car.model().x() < 0)) throw std::runtime_error("S vehicle motion wrong");
    press(Qt::Key_W); expect_command(0, 0, "opposite throttle keys must cancel");
    release(Qt::Key_S); expect_command(1, 0, "release opposite key restores W");
    QFocusEvent lost_focus(QEvent::FocusOut);
    QApplication::sendEvent(&panel, &lost_focus);
    expect_command(0, 0, "focus loss must clear held commands");
    panel.setFocus(); press(Qt::Key_W); expect_command(1, 0, "W after focus return");
    QTest::mouseClick(automatic, Qt::LeftButton);
    wait([&] { return mode == 0 && automatic->isChecked(); }, "auto mode request not published");
    wait([&] { return latest.linear.x == 0 && latest.angular.z == 0; }, "auto mode must clear held commands");
    release(Qt::Key_W);
    QTest::mouseClick(manual, Qt::LeftButton);
    expect_command(0, 0, "returning to manual must not restore stale keys");

    // Render the actual dashboard using real telemetry in wide/narrow modes.
    std_msgs::msg::Float64 value;
    value.data = 1.8; speed->publish(value);
    value.data = 1.0; action->publish(value);
    value.data = 12.4; distance->publish(value);
    std_msgs::msg::String mission; mission.data = "TRANSIT"; stage->publish(mission);
    nav_msgs::msg::Odometry pose;
    pose.pose.pose.position.x = 24.5; pose.pose.pose.position.y = -6.2; odom->publish(pose);
    wait([&] { return panel.findChild<QLabel*>("speedValue")->text() == "1.8"; }, "telemetry not shown");
    for (int i = 0; i < 20; ++i) pump();
    const QString output = qEnvironmentVariable("HUD_PREVIEW_DIR", "hud-preview");
    QDir().mkpath(output);
    auto screenshot = [&](const QString& file, int width, int height) {
      panel.resize(width, height);
      app.processEvents(); QTest::qWait(50);
      if (!panel.grab().save(output + "/" + file)) throw std::runtime_error("preview save failed");
    };
    screenshot("manual-wide.png", 360, 850);
    screenshot("manual-narrow.png", 260, 850);
    QTest::mouseClick(automatic, Qt::LeftButton);
    for (int i = 0; i < 10; ++i) pump();
    screenshot("auto-wide.png", 360, 680);
    screenshot("auto-narrow.png", 260, 680);
    screenshot("auto-short.png", 260, 400);
    std::cout << "PASS: actual Qt WASD events, ROS commands, focus/mode safety, vehicle motion, telemetry and previews\n";
  } catch (const std::exception& error) {
    std::cerr << "FAIL: " << error.what() << '\n';
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
