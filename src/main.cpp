#include <cstdio>
#include <cstdlib>
#include <ctime>

#include "car/car.hpp"

namespace {

constexpr double kDt = 0.1;            // 模拟时间步长（秒）
constexpr int kSteps = 100;            // 模拟步数
constexpr double kCruiseTarget = 2.0;  // 期望巡航速度（m/s）

void print_header() {
  std::printf("========== Autonomous Vehicle Demo ==========\n");
  std::printf("Perception -> Decision -> Control | time step %.1fs, %d steps\n\n", kDt, kSteps);
}

void print_state(const sdc::Car& car, int step) {
  std::printf("[%3d] obstacle: %5.2f m | action: %-10s | speed: %4.2f m/s | distance: %6.2f m\n",
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

  std::printf("\n========== Simulation Complete ==========\n");
  std::printf(
      "total distance: %.2f m | final speed: %.2f m/s\n", car.distance_traveled(), car.speed());
  return 0;
}
