cmake_minimum_required(VERSION 3.16)
project(ROS2DriveNative LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(SDC_ROOT "${CMAKE_CURRENT_LIST_DIR}/.." CACHE PATH "ROS2Drive source root")
set(SDC_SOURCES
  src/scenario/road_network.cpp src/scenario/scenario_definition.cpp
  src/scenario/ring_scenario.cpp src/scenario/mining_haul_scenario.cpp
  src/scenario/port_transport_scenario.cpp src/scenario/agriculture_route_scenario.cpp
  src/simulation/simulation_engine.cpp src/vehicle/simulated_vehicle.cpp
  src/replay/runtime_replay.cpp src/runtime/vehicle_runtime.cpp
  src/control/trajectory_controller.cpp src/control/legacy_controller_adapter.cpp
  src/control/steering_controller.cpp src/control/lqr_controller.cpp
  src/control/mpc_controller.cpp src/mission/mission_manager.cpp
  src/safety/fault_manager.cpp src/safety/safety_manager.cpp
  src/planning/reference_path_planner.cpp src/planning/coverage_path_planner.cpp
  src/planning/ring_lane_planner.cpp src/planning/lattice_planner.cpp
  src/planning/em_planner.cpp src/planning/legacy_planner_adapter.cpp
  src/planning/velocity_planner.cpp src/control/pid_controller.cpp
  src/control/velocity_controller.cpp src/model/ackermann_model.cpp)
list(TRANSFORM SDC_SOURCES PREPEND "${SDC_ROOT}/")
add_library(sdc_runtime_core STATIC ${SDC_SOURCES})
target_include_directories(sdc_runtime_core PUBLIC "${SDC_ROOT}/include")
add_executable(ros2drive_sim "${SDC_ROOT}/native/ros2drive_sim.cpp")
target_link_libraries(ros2drive_sim PRIVATE sdc_runtime_core)
add_executable(ros2drive_live "${SDC_ROOT}/native/ros2drive_live.cpp")
target_link_libraries(ros2drive_live PRIVATE sdc_runtime_core)
enable_testing()
add_test(NAME native_port_smoke COMMAND ros2drive_sim --scenario port_transport --steps 10 --output "${CMAKE_CURRENT_BINARY_DIR}/port-smoke.json")
add_test(NAME native_ring_smoke COMMAND ros2drive_sim --scenario ring_demo --steps 10 --output "${CMAKE_CURRENT_BINARY_DIR}/ring-smoke.json")
add_test(NAME native_mining_smoke COMMAND ros2drive_sim --scenario mining_haul --steps 10 --output "${CMAKE_CURRENT_BINARY_DIR}/mining-smoke.json")
add_test(NAME native_agriculture_smoke COMMAND ros2drive_sim --scenario agriculture_route --steps 10 --output "${CMAKE_CURRENT_BINARY_DIR}/agriculture-smoke.json")

find_package(Python3 COMPONENTS Interpreter QUIET)
if(Python3_Interpreter_FOUND)
  add_test(NAME native_live_protocol
    COMMAND "${Python3_EXECUTABLE}" "${SDC_ROOT}/native/test_live_protocol.py" "$<TARGET_FILE:ros2drive_live>")
endif()
