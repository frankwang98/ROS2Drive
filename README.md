# Self-Driving Car Demo (ROS2 C++)

一个基于 **ROS2 (Humble) + C++** 的自动驾驶小车演示项目，包含：
**感知（Sensor）→ 决策（Decision）→ 控制（Control）** 完整闭环，
并提供**环形道路仿真**，可在 **RViz2** 中实时可视化。

## 功能概览

- **感知层**：距离传感器模拟，检测小车前方障碍物距离（含噪声）
- **决策层**：根据传感器数据决定小车行为（加速 / 匀速 / 减速 / 停车）
- **控制层**：电机控制器，根据决策结果平滑调节速度
- **随机障碍物系统**：在环形道路上随机生成 / 消失障碍物（位置、大小、存活时长随机），RViz 中以红色方块实时显示
- **Lattice 局部规划避障**：基于 lattice 采样生成多条候选局部路径，根据与障碍物的距离与横向偏移评估代价，选择最优避障轨迹，并在 RViz 中显示候选路径（灰）与选中的最优路径（绿）
- **多种速度控制算法**：支持 PID / Bang-Bang / Ramp 三种算法，可通过话题 `/sdc/control_algo` 实时切换
- **阿克曼运动学模型**：小车基于**单车（bicycle）模型**真实转弯行驶（非完整约束），满足 `yaw' = v/L·tan(δ)`；配合 Stanley 转向控制（航向误差 + 前轴横向误差）平滑跟踪期望路径，避免随意斜线滑移与"画龙"抖动
- **自动驾驶仿真节点** (`ring_road_sim`)：内置 **2 张地图**
  - **环形道路（Map 1）**：RViz2 Marker 绘制环形双车道（路面 + 内/外边界 + 中央虚线），小车沿环道避障行驶，HUD 提供「开始 / 暂停」
  - **科目二综合赛道（Map 2）**：倒车入库 / 侧方停车 / 直角转弯 三个科目**在同一条道路上**依次完成，HUD 提供「开始考试」，**无障碍物**
  - 车体 + 速度矢量箭头（颜色随加速/巡航/减速/停车变化）
  - TF 广播 `world → car_base_link`
- **自动驾驶常用可视化控件**（仿真节点内置）：
  - LIDAR 点云（360° 扫描，`/sensor/lidar`，PointCloud2，含障碍物反射）
  - 规划路径（绿色曲线沿环道中心前伸）
  - 行驶轨迹（青色历史轨迹，可清除）
  - 状态 3D 文本（速度 / 行为 / 前方距离 / 控制算法，悬于车顶）
  - 障碍物（`/simulation/obstacles`）与局部规划候选路径（`/simulation/lattice`）
  - HUD 控制话题：`/sdc/speed`、`/sdc/action_id`、`/sdc/front_distance`、`/sdc/start`、`/sdc/pause`、`/sdc/clear_trail`、`/sdc/control_algo`、`/sdc/set_map`、`/sdc/start_exam`、`/sdc/exam_status`
- **RViz 自定义 HUD 面板插件** (`sdc/HudPanel`)：
  - 实时显示速度 / 行为 / 前方距离
  - 切换地图（环形道路 / 科目二综合赛道）
  - 环形道路：开始 / 暂停；科目二：开始考试
  - 随 `ring_road.rviz` 默认加载
- **小车控制器节点** (`car_controller`)：独立 ROS2 节点，订阅距离话题 → 决策 → 控制 → 发布速度

## 目录结构

```
.
├── CMakeLists.txt              # ament_cmake 构建配置
├── package.xml                 # ROS2 包清单
├── include/
│   ├── car/car.hpp             # 小车实体（集成各子系统）
│   ├── sensor/distance_sensor.hpp
│   ├── decision/decision_maker.hpp
│   ├── control/motor_controller.hpp
│   ├── control/pid_controller.hpp      # PID 控制器
│   ├── control/velocity_controller.hpp # 多种速度控制算法（PID/Bang-Bang/Ramp）
│   ├── control/steering_controller.hpp # Stanley 转向控制器
│   ├── model/ackermann_model.hpp       # 阿克曼（单车）运动学模型
│   ├── planning/lattice_planner.hpp    # Lattice 局部规划避障
│   └── sim/obstacle_manager.hpp        # 随机障碍物管理器
├── src/
│   ├── main.cpp                # 旧版单机 demo（不依赖 ROS2）
│   ├── car/car.cpp
│   ├── sensor/distance_sensor.cpp
│   ├── decision/decision_maker.cpp
│   ├── control/motor_controller.cpp
│   ├── control/pid_controller.cpp
│   ├── control/velocity_controller.cpp
│   ├── control/steering_controller.cpp # Stanley 转向控制实现
│   ├── model/ackermann_model.cpp       # 阿克曼运动学模型实现
│   ├── planning/lattice_planner.cpp
│   ├── sim/obstacle_manager.cpp
│   └── ros2/
│       ├── ring_road_sim_node.cpp    # 环形道路仿真 + 可视化 + LIDAR/HUD/避障
│       └── car_controller_node.cpp   # 小车控制器节点
└── src/rviz/
    ├── sdc_hud_panel.hpp / .cpp      # RViz 自定义 HUD 控制面板（可选编译）
    └── plugin_description.xml        # pluginlib 插件描述
├── launch/
│   └── ring_road.launch.py     # 启动仿真 + RViz2
└── rviz/
    └── ring_road.rviz          # RViz2 配置（俯视视角 + HUD 面板）
```

## 环境要求

- Ubuntu 22.04 / 24.04
- ROS2 Humble
- 构建工具：`colcon`、`ament_cmake`
- （可选，RViz HUD 面板需要）`rviz2` 及 Qt5 Widgets

```bash
sudo apt install -y ros-humble-desktop   # 包含 rviz2、rviz 插件等
```

## 构建与运行

```bash
# 1. 配置 ROS2 环境
source /opt/ros/humble/setup.bash

# 2. 构建
colcon build --packages-select self_driving_car_demo
source install/setup.bash

# 3. 启动仿真（自动拉起 RViz2）。可用 map 参数指定初始地图：
#     0=环形道路 1=科目二综合赛道
ros2 launch self_driving_car_demo ring_road.launch.py
ros2 launch self_driving_car_demo ring_road.launch.py map:=1   # 科目二综合赛道
```

启动后即可在 RViz2 中看到（`ring_road.rviz` 已默认加载以下话题与面板，**无需手动订阅**）：
- 灰色环形双车道 + 白色标线（`/simulation/markers`，Transient Local，打开即可见）
- 蓝色小车沿环道行驶 + 速度矢量箭头（颜色随行为变化：绿=加速 / 蓝=巡航 / 橙=减速 / 红=停车）
- 随机红色障碍物方块（`/simulation/obstacles`，随机生成 / 消失）
- Lattice 局部规划：灰色候选路径 + 绿色选中的最优避障路径（`/simulation/lattice`）
- 绿色规划路径 + 青色行驶轨迹 + 车顶 3D 状态文本（`/simulation/markers_live`）
- 360° LIDAR 点云（黄色，`/sensor/lidar`）
- **SDC HUD 面板**（右侧，`sdc/HudPanel`）：实时显示小车状态，切换地图、开始/暂停、开始考试

> 如果使用自定义 RViz 窗口，可手动添加以上话题与面板：
> - `/simulation/markers`（道路，MarkerArray）
> - `/simulation/obstacles`（障碍物，MarkerArray）
> - `/simulation/lattice`（局部规划候选路径，MarkerArray）
> - `/simulation/markers_live`（小车/路径/轨迹/文本）
> - `/sensor/lidar`（LIDAR 点云）
> - 面板：`Panels → Add → New panel → sdc/HudPanel`

## 切换地图与科目二考试

仿真内置 **2 张地图**，功能区分清晰：

| 地图 | 功能 | 按钮 | 障碍物 |
| --- | --- | --- | --- |
| **1. 环形道路** | 原有环形车道行驶 | 「开始 / 暂停」 | 保留随机障碍避障 |
| **2. 科目二综合赛道** | 倒车入库 / 侧方停车 / 直角转弯 三个科目**在同一条道路上**依次完成 | 「开始考试」 | 无障碍物 |

**方式一：RViz HUD 面板**（推荐）
- 右侧 `SDC HUD 面板` 的「地图」下拉框选择：`环形道路` 或 `科目二综合赛道`
- 切到**环形道路**：点「开始」/「暂停」控制行驶
- 切到**科目二综合赛道**：点「开始考试」按钮，小车在同一条赛道上依次完成
  `倒车入库 → 侧方停车 → 直角转弯`，每完成一项在车顶文本与面板显示进度，
  全部完成提示「考试合格」
- 「重置小车」把车放回当前地图起点，「清除轨迹」保持不变

**方式二：话题控制**
```bash
# 切换地图（0=环形道路 1=科目二综合赛道）
ros2 topic pub --once /sdc/set_map std_msgs/msg/Int32 "{data: 1}"
# 开始行驶（环形道路用）
ros2 topic pub --once /sdc/start std_msgs/msg/Bool "{data: true}"
# 暂停/继续行驶
ros2 topic pub --once /sdc/pause std_msgs/msg/Bool "{data: true}"
# 开始科目二考试
ros2 topic pub --once /sdc/start_exam std_msgs/msg/Bool "{data: true}"
# 重置小车到当前地图起点
ros2 topic pub --once /sdc/reset_car std_msgs/msg/Bool "{data: true}"
```

考试状态可通过话题查看：
- `/sdc/exam_status`（`String`，如「考试合格！全部科目完成」）
- `/sdc/exam_progress`（`String`，如「1/3」）
- `/sdc/exam_item`（`Int32`，当前科目序号，-1=无）

> 实现说明：科目二综合赛道（`ExamTrackMap`）把 3 个科目都布置在同一条道路上，
> 通过「航点」依次推进，**不切换地图**。科目二赛道**无障碍物**（`to_obstacles()`
> 返回空），车只沿目标点循迹；环形道路才保留随机障碍避障。
> 倒车入库通过阿克曼模型负车速实现倒车。

## 切换速度控制算法

仿真默认使用 **PID** 控制。运行时可通过话题实时切换（`0=PID 1=Bang-Bang 2=Ramp`）：

```bash
ros2 topic pub --once /sdc/control_algo std_msgs/msg/Int32 "{data: 1}"   # 切到 Bang-Bang
ros2 topic pub --once /sdc/control_algo std_msgs/msg/Int32 "{data: 0}"   # 切回 PID
```

## 独立运行旧版单机 demo（不依赖 ROS2）

```bash
mkdir -p build && cd build
cmake .. && make
./demo_car
```

## 模拟效果示例

```
[感知] 前方障碍物距离: 8.20 m
[决策] 当前行为: 加速
[控制] 目标速度: 2.60 m/s | 当前速度: 2.40 m/s
[状态] 行驶距离: 2.40 m | 当前速度: 2.40 m/s
```

## 后续规划

- [x] 加入 PID 速度控制（含 Bang-Bang / Ramp 多种算法，可实时切换）
- [x] 加入局部路径规划避障（Lattice Planner，RViz 可视化候选与最优路径）
- [x] 随机动态障碍物系统（RViz 实时显示）
- [x] 可切换地图（环形道路 / 科目二综合赛道）
- [x] 科目二模拟考试（3 个科目在同一条道路上，HUD 开始考试，逐项完成并提示）
- [ ] 接入真实传感器（超声波 / 激光雷达）数据
- [ ] 扩展为 Gazebo 物理仿真
