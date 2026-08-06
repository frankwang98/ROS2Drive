# Self-Driving Car Demo (C++)

一个用 C++ 编写的自动驾驶小车演示项目，用于学习/演示自动驾驶小车的基本工作流程：
**感知（Sensor）→ 决策（Decision）→ 控制（Control）**。

## 功能概览

- **感知层**：距离传感器模拟，检测小车前方障碍物距离
- **决策层**：根据传感器数据决定小车行为（加速 / 匀速 / 减速 / 停车）
- **控制层**：电机控制器，根据决策结果调节速度
- **主循环**：按固定时间步长模拟小车运动，并在终端实时打印状态

## 目录结构

```
.
├── CMakeLists.txt              # CMake 构建配置
├── include/
│   ├── car/car.hpp             # 小车实体（集成各子系统）
│   ├── sensor/distance_sensor.hpp
│   ├── decision/decision_maker.hpp
│   └── control/motor_controller.hpp
├── src/
│   ├── main.cpp                # 程序入口与模拟主循环
│   ├── car/car.cpp
│   ├── sensor/distance_sensor.cpp
│   ├── decision/decision_maker.cpp
│   └── control/motor_controller.cpp
└── README.md
```

## 构建与运行

依赖：CMake >= 3.10，支持 C++17 的编译器（g++ / clang++）。

```bash
# 构建
mkdir -p build && cd build
cmake ..
make

# 运行
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

- [ ] 接入真实传感器（超声波 / 激光雷达）数据
- [ ] 加入 PID 速度控制
- [ ] 加入路径规划与避障算法
- [ ] 支持仿真环境（如 Gazebo / Webots）可视化
