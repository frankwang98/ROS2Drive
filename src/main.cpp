#include <cstdio>
#include <cstdlib>
#include <ctime>

#include "car/car.hpp"

namespace {

constexpr double kDt = 0.1;           // 模拟时间步长（秒）
constexpr int kSteps = 100;           // 模拟步数
constexpr double kCruiseTarget = 2.0; // 期望巡航速度（m/s）

void print_header() {
  std::printf("========== 自动驾驶小车 Demo ==========\n");
  std::printf("感知 -> 决策 -> 控制  |  时间步长 %.1fs, 共 %d 步\n\n",
              kDt, kSteps);
}

void print_state(const sdc::Car& car, int step) {
  std::printf("[%3d] 障碍物: %5.2f m | 行为: %-6s | 速度: %4.2f m/s | 行驶: %6.2f m\n",
              step,
              car.front_distance(),
              sdc::action_name(car.current_action()),
              car.speed(),
              car.distance_traveled());
}

}  // namespace

int main() {
  std::srand(static_cast<unsigned>(std::time(nullptr)));

  sdc::Car car;
  print_header();

  for (int i = 1; i <= kSteps; ++i) {
    car.step(kDt);
    print_state(car, i);
  }

  std::printf("\n========== 模拟结束 ==========\n");
  std::printf("总行驶距离: %.2f m | 最终速度: %.2f m/s\n",
              car.distance_traveled(), car.speed());
  return 0;
}
