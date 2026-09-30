#include "vehicle/simulated_vehicle.hpp"
#include "control/velocity_controller.hpp"
#include <iostream>
#include <stdexcept>

int main() {
  try {
    for (auto algorithm : {sdc::VelocityAlgorithm::kPid, sdc::VelocityAlgorithm::kBangBang,
                           sdc::VelocityAlgorithm::kRamp}) {
      sdc::VelocityController controller;
      controller.set_algorithm(algorithm);
      double speed = 0.0;
      for (int i = 0; i < 80; ++i) speed = controller.update(-2.0, speed, 0.05, true);
      if (speed >= -0.1) throw std::runtime_error("signed manual velocity must reverse");
      controller.reset();
      speed = 0.0;
      for (int i = 0; i < 80; ++i) speed = controller.update(2.0, speed, 0.05);
      if (speed <= 0.1) throw std::runtime_error("autonomous forward velocity changed");
    }
    for (auto algorithm : {sdc::VelocityAlgorithm::kPid, sdc::VelocityAlgorithm::kRamp}) {
      sdc::VelocityController controller;
      controller.set_algorithm(algorithm);
      if (controller.update(-2.0, 0.0, 0.05) < 0.0)
        throw std::runtime_error("default forward-only clamp changed");
    }
    sdc::vehicle::SimulatedVehicle car;
    car.reset(0, 0, 0);
    for (int i = 0; i < 40; ++i) car.applyManual(-1, 0, 0.05);
    if (car.model().x() >= 0 || car.model().speed() >= 0)
      throw std::runtime_error("manual reverse does not move backwards");
    std::cout << "PASS: manual reverse motion and autonomous velocity compatibility\n";
  } catch (const std::exception& e) { std::cerr << e.what() << '\n'; return 1; }
}
