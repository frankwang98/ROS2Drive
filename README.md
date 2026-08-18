# Self-Driving Car Demo (ROS2 C++)

一个基于 **ROS2 (Humble) + C++** 的自动驾驶小车演示项目，包含：
**感知（Sensor）→ 决策（Decision）→ 控制（Control）** 完整闭环，
并提供**复杂环形道路仿真**，可在 **RViz2** 中实时可视化。

## 功能概览

- **感知层**：距离传感器模拟，检测小车前方障碍物距离（含噪声）
- **决策层**：根据传感器数据决定小车行为（加速 / 匀速 / 减速 / 停车）
- **控制层**：电机控制器，根据决策结果平滑调节速度
- **复杂环形道路**：在基础环形基础上增加道路复杂性——
  - **S 形绕桩路段**：一排交替内/外侧桩桶，车辆需蛇形穿梭
  - **窄门路段**：两侧收窄形成门形通道，考验居中控制
  - **减速带 / 静态路障**：部分路段设置固定障碍，需绕行
  - **多车道标线 / 双黄线 / 人行横道** 等路面细节增强路感
- **随机障碍物系统**：在环形道路上随机生成 / 消失障碍物（位置、大小、存活时长随机），RViz 中以红色方块实时显示
- **Lattice 局部规划避障**：基于 lattice 采样生成多条候选局部路径，根据与障碍物的距离与横向偏移评估代价，选择最优避障轨迹，并在 RViz 中显示候选路径（灰）与选中的最优路径（绿）
- **EM 局部规划避障**：基于 Frenet 框架 + EM 思想（E 步采样 → M 步选择），用五阶多项式生成光滑横向过渡路径，综合障碍物/横向偏移/平滑代价选优，可与 Lattice 算法实时切换
- **多种速度控制算法**：支持 PID / Bang-Bang / Ramp 三种算法，可通过话题 `/sdc/control_algo` 实时切换
- **多种横向（转向）控制算法**：支持 Stanley / LQR / MPC 三种算法，可通过话题 `/sdc/lateral_algo` 实时切换（LQR 基于线性化车辆模型 + 离散黎卡提方程，MPC 基于滚动时域投影梯度优化）
- **行为树基础行为切换**：基于 **BehaviorTree.CPP v3** 实现基础行为（加速 / 匀速巡航 / 减速 / 停车）的树形切换，行为通过 XML 描述、条件节点 + 行为节点在黑板中传递决策，可通过话题 `/sdc/behavior_tree` 与规则决策实时切换
- **阿克曼运动学模型**：小车基于**单车（bicycle）模型**真实转弯行驶（非完整约束），满足 `yaw' = v/L·tan(δ)`；配合 Stanley 转向控制平滑跟踪期望路径
- **自动驾驶仿真节点** (`ring_road_sim`)：单张环形地图，支持 **自动 / 手动** 两种驾驶模式：
  - **自动**：Lattice 避障 + Stanley 循迹，沿环形道路自主行驶
  - **手动**：通过 **WASD 键盘**直接控制（W=前进  S=倒车  A=左转  D=右转）
  - 车体 + 速度矢量箭头（颜色随加速/巡航/减速/停车变化）
  - TF 广播 `world → car_base_link`
- **自动驾驶常用可视化控件**（仿真节点内置）：
  - LIDAR 点云（360° 扫描，`/sensor/lidar`，PointCloud2，含障碍物反射）
  - 规划路径（绿色曲线沿环道中心前伸）
  - 行驶轨迹（青色历史轨迹，可清除）
  - 状态 3D 文本（模式 / 速度 / 行为，悬于车顶）
  - 障碍物（`/simulation/obstacles`）与局部规划候选路径（`/simulation/lattice`）
  - HUD 控制话题：`/sdc/speed`、`/sdc/action_id`、`/sdc/front_distance`、`/sdc/mode`、`/sdc/set_mode`、`/sdc/manual_cmd`、`/sdc/start`、`/sdc/pause`、`/sdc/clear_trail`、`/sdc/control_algo`、`/sdc/planning_algo`、`/sdc/lateral_algo`、`/sdc/behavior_tree`
- **RViz 自定义 HUD 面板插件** (`sdc/HudPanel`)：
  - 实时显示速度 / 行为 / 前方距离 / 驾驶模式
  - **自动 / 手动驾驶模式切换**
  - 手动模式下 **WASD 键盘控制**
  - 开始 / 暂停、清除轨迹、重置小车
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
│   ├── control/lqr_controller.hpp      # LQR 横向控制器（线性化车辆模型 + DARE）
│   ├── control/mpc_controller.hpp      # MPC 横向控制器（滚动时域优化）
│   ├── model/ackermann_model.hpp       # 阿克曼（单车）运动学模型
│   ├── planning/lattice_planner.hpp    # Lattice 局部规划避障
│   ├── planning/em_planner.hpp         # EM 局部规划避障（Frenet + EM 采样）
│   ├── behavior_tree/behavior_tree_planner.hpp # 行为树基础行为切换（BehaviorTree.CPP v3）
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
│   ├── control/lqr_controller.cpp      # LQR 转向控制实现
│   ├── control/mpc_controller.cpp      # MPC 转向控制实现
│   ├── model/ackermann_model.cpp       # 阿克曼运动学模型实现
│   ├── planning/lattice_planner.cpp
│   ├── planning/em_planner.cpp         # EM 局部规划实现
│   ├── behavior_tree/behavior_tree_planner.cpp # 行为树实现
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

# 3. 启动仿真（自动拉起 RViz2）。可用 mode 参数指定初始驾驶模式：
#     0=自动 1=手动(WASD)
ros2 launch self_driving_car_demo ring_road.launch.py
ros2 launch self_driving_car_demo ring_road.launch.py mode:=1   # 手动驾驶启动
```

启动后即可在 RViz2 中看到（`ring_road.rviz` 已默认加载以下话题与面板，**无需手动订阅**）：
- 灰色环形双车道 + 白/黄标线 + 复杂路况（`/simulation/markers`，Transient Local，打开即可见）
- 蓝色小车沿环道行驶 + 速度矢量箭头（颜色随行为变化：绿=加速 / 蓝=巡航 / 橙=减速 / 红=停车）
- 随机红色障碍物方块（`/simulation/obstacles`，随机生成 / 消失）
- Lattice 局部规划：灰色候选路径 + 绿色选中的最优避障路径（`/simulation/lattice`）
- 绿色规划路径 + 青色行驶轨迹 + 车顶 3D 状态文本（`/simulation/markers_live`）
- 360° LIDAR 点云（黄色，`/sensor/lidar`）
- **SDC HUD 面板**（右侧，`sdc/HudPanel`）：实时显示小车状态、切换自动/手动驾驶

> 如果使用自定义 RViz 窗口，可手动添加以上话题与面板：
> - `/simulation/markers`（道路，MarkerArray）
> - `/simulation/obstacles`（障碍物，MarkerArray）
> - `/simulation/lattice`（局部规划候选路径，MarkerArray）
> - `/simulation/markers_live`（小车/路径/轨迹/文本）
> - `/sensor/lidar`（LIDAR 点云）
> - 面板：`Panels → Add → New panel → sdc/HudPanel`

## 自动 / 手动驾驶

仿真内置 **2 种驾驶模式**，可随时切换：

| 模式 | 说明 | 控制方式 |
| --- | --- | --- |
| **自动** | Lattice 局部避障 + Stanley 循迹，沿复杂环形道路自主行驶，自动绕开桩桶/窄门/路障 | 无需操作 |
| **手动 (WASD)** | 通过键盘直接控制车辆 | W=前进  S=倒车  A=左转  D=右转 |

**方式一：RViz HUD 面板**（推荐）
- 右侧 `SDC HUD 面板` 点击「切换到手动 (WASD)」进入手动模式
- 手动模式下，**点击面板使其获得键盘焦点**，然后按住 `W/S/A/D` 控制小车
- 松开按键即松开油门 / 回正转向；点击「切换到自动」回到自动驾驶

**方式二：话题控制**
```bash
# 切换到手动驾驶
ros2 topic pub --once /sdc/set_mode std_msgs/msg/Int32 "{data: 1}"
# 切换到自动驾驶
ros2 topic pub --once /sdc/set_mode std_msgs/msg/Int32 "{data: 0}"
# 手动控制指令（W=前进 throttle=+1，S=倒车 throttle=-1，A=左转 steer=-1，D=右转 steer=+1）
ros2 topic pub --rate 20 /sdc/manual_cmd geometry_msgs/msg/Twist "{linear: {x: 1.0}, angular: {z: 0.0}}"
# 开始行驶 / 暂停
ros2 topic pub --once /sdc/start std_msgs/msg/Bool "{data: true}"
ros2 topic pub --once /sdc/pause std_msgs/msg/Bool "{data: true}"
# 重置小车到环形起点
ros2 topic pub --once /sdc/reset_car std_msgs/msg/Bool "{data: true}"
```

驾驶模式可通过 `/sdc/mode`（`Int32`，0=自动 1=手动）话题查看。

## 切换速度控制算法

仿真默认使用 **PID** 控制。运行时可通过话题实时切换（`0=PID 1=Bang-Bang 2=Ramp`）：

```bash
ros2 topic pub --once /sdc/control_algo std_msgs/msg/Int32 "{data: 1}"   # 切到 Bang-Bang
ros2 topic pub --once /sdc/control_algo std_msgs/msg/Int32 "{data: 0}"   # 切回 PID
```

## 切换局部规划 / 横向控制算法

仿真默认使用 **Lattice 局部规划 + Stanley 横向控制**。运行时可通过话题实时切换，便于对比学习各算法表现：

```bash
# 局部规划算法（0=Lattice 1=EM）
ros2 topic pub --once /sdc/planning_algo std_msgs/msg/Int32 "{data: 1}"   # 切到 EM Planner
ros2 topic pub --once /sdc/planning_algo std_msgs/msg/Int32 "{data: 0}"   # 切回 Lattice

# 横向控制算法（0=Stanley 1=LQR 2=MPC）
ros2 topic pub --once /sdc/lateral_algo std_msgs/msg/Int32 "{data: 1}"   # 切到 LQR
ros2 topic pub --once /sdc/lateral_algo std_msgs/msg/Int32 "{data: 2}"   # 切到 MPC
ros2 topic pub --once /sdc/lateral_algo std_msgs/msg/Int32 "{data: 0}"   # 切回 Stanley

# 基础行为决策（0=规则决策 1=行为树）
ros2 topic pub --once /sdc/behavior_tree std_msgs/msg/Int32 "{data: 1}"  # 切到行为树 (BehaviorTree.CPP v3)
ros2 topic pub --once /sdc/behavior_tree std_msgs/msg/Int32 "{data: 0}"  # 切回规则决策
```

当前生效的规划/控制算法会显示在车顶 3D 状态文本中（如 `自动 Lattice/Stanley`）。各算法实现位于 `include/planning/` 与 `include/control/`，参数（视距、采样数、Q/R 权重等）可直接在对应头文件中调整，方便学习参数对控制效果的影响。

行为树基于 **BehaviorTree.CPP v3**（可选依赖）。默认规则决策不依赖该库；安装后可启用完整行为树实现：

```bash
sudo apt install -y ros-humble-behaviortree-cpp-v3
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
- [x] 加入 EM Planner（Frenet + EM 采样，可与 Lattice 实时切换）
- [x] 加入横向控制算法（Stanley / LQR / MPC，可实时切换）
- [x] 随机动态障碍物系统（RViz 实时显示）
- [x] 环形道路增加复杂性（S 形绕桩 / 窄门 / 路障 / 多车道标线）
- [x] 自动 / 手动（WASD）双驾驶模式
- [x] 行为树基础行为切换（BehaviorTree.CPP v3，可选依赖）
- [ ] 接入真实传感器（超声波 / 激光雷达）数据
- [ ] 扩展为 Gazebo 物理仿真
