# Self-Driving Car Demo (ROS2 C++)

一个基于 **ROS2 (Humble) + C++** 的自动驾驶小车演示项目，包含：
**感知（Sensor）→ 决策（Decision）→ 控制（Control）** 完整闭环，
并提供**环形道路仿真**，可在 **RViz2** 中实时可视化。

## 功能概览

- **感知层**：距离传感器模拟，检测小车前方障碍物距离（含噪声）
- **决策层**：根据传感器数据决定小车行为（加速 / 匀速 / 减速 / 停车）
- **控制层**：电机控制器，根据决策结果平滑调节速度
- **环形道路仿真节点** (`ring_road_sim`)：
  - RViz2 Marker 绘制环形双车道（路面 + 内/外边界 + 中央虚线）
  - 小车沿环道行驶，动态障碍物驱动感知→决策→控制闭环
  - 车体 + 速度矢量箭头（颜色随加速/巡航/减速/停车变化）
  - TF 广播 `world → car_base_link`
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
│   └── control/motor_controller.hpp
├── src/
│   ├── main.cpp                # 旧版单机 demo（不依赖 ROS2）
│   ├── car/car.cpp
│   ├── sensor/distance_sensor.cpp
│   ├── decision/decision_maker.cpp
│   ├── control/motor_controller.cpp
│   └── ros2/
│       ├── ring_road_sim_node.cpp    # 环形道路仿真 + RViz2 可视化
│       └── car_controller_node.cpp   # 小车控制器节点
├── launch/
│   └── ring_road.launch.py     # 启动仿真 + RViz2
└── rviz/
    └── ring_road.rviz          # RViz2 配置（俯视视角）
```

## 环境要求

- Ubuntu 22.04 / 24.04
- ROS2 Humble
- 构建工具：`colcon`、`ament_cmake`

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

# 3. 启动环形道路仿真（自动拉起 RViz2）
ros2 launch self_driving_car_demo ring_road.launch.py
```

启动后即可在 RViz2 中看到：
- 灰色环形双车道 + 白色标线
- 蓝色小车沿环道行驶
- 速度矢量箭头（颜色随行为变化：绿=加速 / 蓝=巡航 / 橙=减速 / 红=停车）

> 如果已有 RViz2 窗口，可手动订阅话题 `/simulation/markers`（MarkerArray）。

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

- [ ] 加入 PID 速度控制
- [ ] 加入路径规划与避障算法
- [ ] 接入真实传感器（超声波 / 激光雷达）数据
- [ ] 扩展为 Gazebo 物理仿真
