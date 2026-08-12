cmake_minimum_required(VERSION 3.10)
project(self_driving_car_demo LANGUAGES CXX)

# ============================================================
# ROS2 / ament 构建
# ============================================================
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

# RViz 自定义面板插件（可选依赖）
# 基础环境缺少 rviz_common / Qt5 时自动跳过，不影响核心仿真构建
find_package(rviz_common QUIET)
find_package(Qt5 QUIET COMPONENTS Widgets)
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

# ---- 核心算法库（纯逻辑，不依赖 ROS）----
add_library(sdc_core STATIC
  src/sensor/distance_sensor.cpp
  src/decision/decision_maker.cpp
  src/control/motor_controller.cpp
  src/control/pid_controller.cpp
  src/control/velocity_controller.cpp
  src/planning/lattice_planner.cpp
  src/sim/obstacle_manager.cpp
  src/car/car.cpp
)
target_include_directories(sdc_core PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# ---- 旧版单机 demo（保留，方便对比）----
add_executable(demo_car
  src/main.cpp
  src/car/car.cpp
)
target_link_libraries(demo_car sdc_core)
target_include_directories(demo_car PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# ---- ROS2 仿真 + 可视化节点 ----
add_executable(ring_road_sim
  src/ros2/ring_road_sim_node.cpp
)
ament_target_dependencies(ring_road_sim
  rclcpp
  std_msgs
  visualization_msgs
  geometry_msgs
  sensor_msgs
  tf2
  tf2_ros
)
target_link_libraries(ring_road_sim sdc_core)
target_include_directories(ring_road_sim PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# ---- ROS2 小车控制器节点 ----
add_executable(car_controller
  src/ros2/car_controller_node.cpp
)
ament_target_dependencies(car_controller
  rclcpp
  std_msgs
  visualization_msgs
  geometry_msgs
)
target_link_libraries(car_controller sdc_core)
target_include_directories(car_controller PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# ---- RViz 自定义 HUD 面板插件（可选）----
if(SDC_BUILD_RVIZ_PANEL)
  find_package(pluginlib REQUIRED)
  qt5_wrap_cpp(sdc_hud_panel_MOC
    src/rviz/sdc_hud_panel.hpp
  )
  add_library(sdc_hud_panel SHARED
    src/rviz/sdc_hud_panel.cpp
    ${sdc_hud_panel_MOC}
  )
  target_link_libraries(sdc_hud_panel
    Qt5::Widgets
  )
  ament_target_dependencies(sdc_hud_panel
    rclcpp
    std_msgs
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
  install(FILES src/rviz/plugin_description.xml
    DESTINATION share/${PROJECT_NAME}
  )
  pluginlib_export_plugin_description_file(rviz_common
    src/rviz/plugin_description.xml
  )
endif()

# ---- 安装 ----
install(TARGETS
  demo_car
  ring_road_sim
  car_controller
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch/ rviz/
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
