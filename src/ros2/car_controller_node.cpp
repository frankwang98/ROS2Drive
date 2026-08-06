/**
 * car_controller_node.cpp — 小车控制器节点（可选独立运行）
 *
 * 订阅传感器话题 → 决策 → 控制 → 发布速度指令。
 * 可作为独立节点接入更大的 ROS2 系统。
 */

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "car/car.hpp"

class CarControllerNode : public rclcpp::Node {
public:
  CarControllerNode()
      : Node("car_controller")
  {
    sub_distance_ = create_subscription<std_msgs::msg::Float64>(
        "sensor/distance", 10,
        std::bind(&CarControllerNode::on_distance, this, std::placeholders::_1));

    pub_speed_ = create_publisher<std_msgs::msg::Float64>("actuator/speed", 10);
    pub_action_ = create_publisher<std_msgs::msg::Float64>("car/action_id", 10);

    timer_ = create_wall_timer(
        std::chrono::milliseconds(50),
        std::bind(&CarControllerNode::control_step, this));

    RCLCPP_INFO(get_logger(), "小车控制器就绪");
  }

private:
  void on_distance(const std_msgs::msg::Float64::SharedPtr msg) {
    front_distance_ = msg->data;
  }

  void control_step() {
    // 感知 → 决策 → 控制
    sdc::Action action = decision_.decide(front_distance_);
    double speed = motor_.update(action, car_speed_, 0.05);
    car_speed_ = speed;

    // 发布速度指令
    auto speed_msg = std_msgs::msg::Float64();
    speed_msg.data = speed;
    pub_speed_->publish(speed_msg);

    // 发布行为 ID（方便其他节点消费）
    auto action_msg = std_msgs::msg::Float64();
    action_msg.data = static_cast<double>(static_cast<int>(action));
    pub_action_->publish(action_msg);
  }

  sdc::DecisionMaker  decision_;
  sdc::MotorController motor_;
  double front_distance_{30.0};
  double car_speed_{0.0};

  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr sub_distance_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_speed_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_action_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<CarControllerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
