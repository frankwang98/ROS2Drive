cmake_minimum_required(VERSION 3.10)
# rosidl generates both C and C++ support libraries even though Runtime itself
# is C++. Keep C enabled or the generated *_rosidl_generator_c target has no
# build rule on ROS 2 Jazzy/Humble.
project(self_driving_car_demo LANGUAGES C CXX)

# ============================================================
# ROS2 / ament 构建
# ============================================================
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/TrajectoryPoint.msg"
  "msg/Trajectory.msg"
  "msg/ControlCommand.msg"
  "msg/Fault.msg"
  "msg/FaultArray.msg"
  "msg/RuntimeStatus.msg"
  "msg/RuntimeMetrics.msg"
  "action/ExecuteMission.action"
  DEPENDENCIES builtin_interfaces std_msgs geometry_msgs
)

# RViz 自定义面板插件（可选依赖）
# 基础环境缺少 rviz_common / Qt5 时自动跳过，不影响核心仿真构建
find_package(rviz_common QUIET)
find_package(Qt5 QUIET COMPONENTS Widgets)

# 行为树（可选）：基于 BehaviorTree.CPP v3 实现基础行为切换。
# 若环境已安装 behaviors_cpp_v3 则启用完整行为树实现，否则自动退化为规则决策。
find_package(behaviortree_cpp_v3 QUIET)
if(behaviortree_cpp_v3_FOUND)
  set(SDC_BUILD_BEHAVIOR_TREE ON)
  message(STATUS "BehaviorTree.CPP v3: ENABLED (行为树实现)")
else()
  set(SDC_BUILD_BEHAVIOR_TREE OFF)
  message(STATUS "BehaviorTree.CPP v3: DISABLED (退化为规则决策)")
endif()
if(rviz_common_FOUND AND Qt5Widgets_FOUND)
  set(SDC_BUILD_RVIZ_PANEL ON)
  message(STATUS "RViz HUD panel plugin: ENABLED")
else()
  set(SDC_BUILD_RVIZ_PANEL OFF)
  message(STATUS "RViz HUD panel plugin: DISABLED (rviz_common/Qt5 not found)")
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

# ---- 场景无关 Runtime 核心（严格不依赖 ROS）----
add_library(sdc_runtime_core STATIC
  src/scenario/road_network.cpp
  src/scenario/scenario_definition.cpp
  src/scenario/ring_scenario.cpp
  src/scenario/mining_haul_scenario.cpp
  src/scenario/port_transport_scenario.cpp
  src/scenario/agriculture_route_scenario.cpp
  src/simulation/simulation_engine.cpp
  src/vehicle/simulated_vehicle.cpp
  src/replay/runtime_replay.cpp
  src/runtime/vehicle_runtime.cpp
  src/control/trajectory_controller.cpp
  src/control/legacy_controller_adapter.cpp
  src/control/steering_controller.cpp
  src/control/lqr_controller.cpp
  src/control/mpc_controller.cpp
  src/mission/mission_manager.cpp
  src/safety/fault_manager.cpp
  src/safety/safety_manager.cpp
  src/planning/reference_path_planner.cpp
  src/planning/coverage_path_planner.cpp
  src/planning/ring_lane_planner.cpp
  src/planning/lattice_planner.cpp
  src/planning/em_planner.cpp
  src/planning/legacy_planner_adapter.cpp
  src/planning/velocity_planner.cpp
  src/control/pid_controller.cpp
  src/control/velocity_controller.cpp
  src/model/ackermann_model.cpp
)
target_include_directories(sdc_runtime_core PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# ---- ROS / visualization adapters for the teaching scenarios ----
# 旧 Lattice/EM 及旧横向控制器带有环道几何假设，
# 仅保留源码供对照，不进入正式 Runtime 或默认产物。
add_library(sdc_core STATIC
  src/legacy/sensor/distance_sensor.cpp
  src/legacy/decision/decision_maker.cpp
  src/control/motor_controller.cpp
  src/adapters/simulation/obstacle_manager.cpp
  src/behavior_tree/behavior_tree_planner.cpp
  src/behavior_tree/behavior_tree_behavior.cpp
  src/adapters/visualization/map.cpp
  src/legacy/car/car.cpp
)
target_include_directories(sdc_core PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)
target_link_libraries(sdc_core sdc_runtime_core)
# map.cpp / map.hpp 依赖 visualization_msgs 与 geometry_msgs，
# 必须把对应 ROS 包的头文件路径传给 sdc_core（否则 colcon 找不到相关头文件）。
ament_target_dependencies(sdc_core
  visualization_msgs
  geometry_msgs
)

# 行为树（可选）
if(SDC_BUILD_BEHAVIOR_TREE)
  target_compile_definitions(sdc_core PUBLIC BEHAVIORTREE_CPP_V3_FOUND)
  ament_target_dependencies(sdc_core behaviortree_cpp_v3)
endif()

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(sdc_runtime_tests test/runtime_core_test.cpp)
  target_link_libraries(sdc_runtime_tests sdc_runtime_core)
  target_include_directories(sdc_runtime_tests PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
endif()

# ---- 旧版单机 demo（保留，方便对比）----
add_executable(demo_car
  src/apps/demo_car_main.cpp
  src/legacy/car/car.cpp
)
target_link_libraries(demo_car sdc_core)
target_include_directories(demo_car PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# ---- ROS2 仿真 + 可视化节点 ----
add_executable(ring_road_sim
  src/adapters/ros/nodes/ring_road_sim_node.cpp
)
rosidl_get_typesupport_target(sdc_rosidl_typesupport ${PROJECT_NAME} "rosidl_typesupport_cpp")
add_library(sdc_ros_adapter STATIC src/adapters/ros/runtime_message_converter.cpp)
add_dependencies(sdc_ros_adapter ${PROJECT_NAME}__rosidl_generator_cpp)
target_include_directories(sdc_ros_adapter PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(sdc_ros_adapter sdc_runtime_core ${sdc_rosidl_typesupport})
ament_target_dependencies(sdc_ros_adapter builtin_interfaces std_msgs geometry_msgs)
target_link_libraries(ring_road_sim sdc_ros_adapter ${sdc_rosidl_typesupport})
ament_target_dependencies(ring_road_sim
  rclcpp
  rclcpp_action
  std_msgs
  std_srvs
  visualization_msgs
  geometry_msgs
  sensor_msgs
  nav_msgs
  tf2
  tf2_ros
)
target_link_libraries(ring_road_sim sdc_core)
target_include_directories(ring_road_sim PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# ---- RViz 自定义 HUD 面板插件（可选）----
if(SDC_BUILD_RVIZ_PANEL)
  find_package(pluginlib REQUIRED)
  qt5_wrap_cpp(sdc_hud_panel_MOC
    src/adapters/visualization/rviz/sdc_hud_panel.hpp
  )
  add_library(sdc_hud_panel SHARED
    src/adapters/visualization/rviz/sdc_hud_panel.cpp
    ${sdc_hud_panel_MOC}
  )
  target_link_libraries(sdc_hud_panel
    Qt5::Widgets
  )
  ament_target_dependencies(sdc_hud_panel
    rclcpp
    std_msgs
    geometry_msgs
    nav_msgs
    rviz_common
  )
  target_compile_definitions(sdc_hud_panel PRIVATE QT_NO_KEYWORDS)
  target_include_directories(sdc_hud_panel PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src
  )
  install(TARGETS sdc_hud_panel
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
  )
  install(FILES src/adapters/visualization/rviz/plugin_description.xml
    DESTINATION share/${PROJECT_NAME}
  )
  pluginlib_export_plugin_description_file(rviz_common
    src/adapters/visualization/rviz/plugin_description.xml
  )
endif()

# ---- 安装 ----
install(TARGETS
  sdc_runtime_core
  sdc_core
  sdc_ros_adapter
  demo_car
  ring_road_sim
  DESTINATION lib/${PROJECT_NAME}
)

# Preserve resource directory names. A trailing slash here would flatten all
# files into share/${PROJECT_NAME}, while launch resolves config/... and
# rviz/... beneath the package share directory.
install(DIRECTORY launch rviz config docs
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
