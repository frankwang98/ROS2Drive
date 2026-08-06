cmake_minimum_required(VERSION 3.10)
project(self_driving_car_demo LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

add_executable(demo_car
  src/main.cpp
  src/car/car.cpp
  src/sensor/distance_sensor.cpp
  src/decision/decision_maker.cpp
  src/control/motor_controller.cpp
)

target_include_directories(demo_car PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)
