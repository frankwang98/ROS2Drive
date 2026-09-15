# Low-Speed Autonomy Runtime (ROS 2 / C++)

面向矿山、港口、农业等低速无人车辆的**场景无关 Autonomy Runtime 参考实现**。环形道路只是默认教学场景，不是运行时架构的边界。

当前仓库正在从单体环形 Demo 迁移为可复用 Runtime。新代码遵循“领域层不依赖 ROS、场景通过数据与配置接入、安全层拥有最终控制权”；原 `ring_road_sim` 在迁移完成前继续可运行。

详细资料：[架构与时序](docs/architecture.md) · [ROS 接口契约](docs/interfaces.md) ·
[场景与故障注入](docs/scenarios.md) · [实施路线](TODO.md)

## 系统边界

```text
Mission / Route / Vehicle State / Obstacles
                    │
          ┌─────────▼──────────┐
          │   VehicleRuntime   │  ROS-free, deterministic step()
          │ MissionManager     │
          │ Behavior / Planner │  external XML / generic path
          │ VelocityPlanner    │  curvature/accel/decel limits
          │ Controller         │  replaceable trajectory tracker
          │ SafetyManager      │  final command arbitration
          └─────────┬──────────┘
                    │ Trajectory / Control / State / Faults
          ┌─────────▼──────────┐
          │    ROS Adapter     │  topics/actions/QoS/namespace only
          └─────────┬──────────┘
                    │
       Simulator or real vehicle interface
```

- `domain/`：统一领域对象，不依赖 ROS、地图或具体车型。
- `runtime/`：固定周期编排任务、规划、控制与安全，不处理 ROS callback。
- `mission/`：任务生命周期；云端下发任务，而不是操纵规划器内部参数。
- `planning/`：消费参考路径、车辆状态、障碍物，输出统一稠密轨迹。
- `safety/`：检测超时、定位、规划、急停等故障，并覆盖不安全指令。
- `behavior_tree/`：矿山、港口、农业、教学场景通过外部 XML 配置。
- `ros2/`：唯一 ROS 边界；`sim/`：场景、传感器和车辆模型适配器。
- `ros/`：独立 Domain→ROS typed message converter，避免 Runtime 和仿真节点手写协议映射。
- `simulation/`：ROS-free 教学仿真执行器，组合车辆模型与同一个 Runtime；不包含地图绘制或 ROS 发布。
- `scenario/`：ROS-free 场景合同，统一初始位姿、路线、障碍、默认 Mission、车辆约束和 Behavior Profile。
- `vehicle/`：底盘边界；`VehicleInterface` 统一状态读取、控制执行和健康检查，仿真由 `SimulatedVehicle` 实现，真实 CAN/线控底盘实现相同接口。

## 统一领域接口

```cpp
runtime.setMission(mission);
runtime.updateVehicleState(state);
runtime.updateObstacles(obstacles);
RuntimeOutput output = runtime.step(now_s, dt_s);
```

规划器统一实现 `planning::Planner::plan(PlanningInput)`。几何规划不再决定最终车速，`VelocityPlanner` 根据任务限速、曲率和纵向加减速约束生成速度剖面，然后由可替换的 `Controller` 消费完整 `Trajectory`。输出是车辆可直接跟踪的稠密 waypoint 集：

```text
Waypoint = pose(x,y,yaw) + curvature + velocity
         + acceleration + relative_time
```

当前 `ReferencePathPlanner` 是场景无关基线：对任意参考路径增密、限制前视距离、计算航向/曲率并执行基础局部绕行。`PurePursuitController` 是默认控制器，通过接口可替换实现。旧 Lattice/EM 及旧横向控制器仍依赖圆环假设或旧轨迹类型，源码仅供教学对照，已从默认构建和正式能力面移除。

## 运行状态与任务状态

```text
Runtime: INIT → READY → RUNNING ⇄ PAUSED → STOPPED
                            └→ DEGRADED / FAULT / ESTOP

Mission: PENDING → ACTIVE ⇄ PAUSED → SUCCEEDED
                         ├→ FAILED
                         └→ CANCELED
```

任务类型预留 `NavigateTo / FollowRoute / Stop / Park / Dock / ReturnHome`。当前已实现参数校验、忙时拒绝、显式抢占、开始、暂停、恢复、取消、超时、进度、结果原因和终态。`NavigateTo`/非闭合 `FollowRoute` 到达容差范围后成功，闭合 `FollowRoute` 持续循环，`Stop` 在车速降至阈值后成功。后续 ROS Action 适配不改变领域接口。

## 安全机制

`SafetyManager` 每周期最后执行，planner/controller 不能绕过。已实现定位有效性、车辆状态 freshness、感知 freshness、车辆状态有限值、规划失败、控制输出有限值、软件急停、故障去重/恢复清除，以及异常时强制安全动作。Runtime 一旦收到带时间戳的感知流，就持续检查其 freshness；尚未连接感知源时不会把“空障碍物集合”误判为断流。

每类故障通过 `SafetyConfig::policies` 映射为 `REPORT_ONLY / DEGRADE / STOP / EMERGENCY_STOP`。降级默认限制到低速，停车会进入 `FAULT`，急停进入 `ESTOP`。STOP/ESTOP 动作会锁存：故障条件消失、车辆静止且连续健康达到 `safety.recovery_healthy_cycles` 后，仍需调用 `runtime/acknowledge_recovery` (`std_srvs/Trigger`) 才能恢复；硬件急停的物理复位不由该服务替代。策略由 `runtime.yaml` 的 `safety.policies` 加载，只接受 `report/degrade/stop/estop`，未知值会拒绝启动。
SafetyManager 本身也会拒绝非正的 timeout 或零恢复周期，不依赖 ROS adapter 才能保证配置不变式。

故障等级为 `INFO/WARNING/ERROR/FATAL`。硬件急停与底盘独立安全链路仍由车辆接口层保证，软件 Runtime 不能替代它。

## 行为树与场景适配

外部行为树统一位于 `config/behavior_trees/scene_driving.xml`，由 `DistancePolicy`
参数化策略子树与四个 Behavior Profile（`RingDemo` / `PortTransport` / `MiningHaul` /
`AgricultureRoute`）组成。同一棵子树，仅以 `stop_t` / `slow_t` / `cruise_t` 三个参数
区分场景阈值；节点层不持有任何硬编码默认值，缺少参数会立即失败（fail fast）。
XML 和 C++ API 明确使用 **BehaviorTree.CPP v3**，不混用 v4 的 `BTCPP_format="4"`
声明。`BehaviorTreePlanner::init(xml_path, tree_id)` 加载指定文件和树；未安装 BehaviorTree.CPP v3 时退化为
规则决策。

默认 launch 将 `scene_driving.xml` 通过 `BehaviorTreeBehavior` 注入 Runtime。可通过
`bt_xml` 参数替换为其他 XML，并通过 `bt_tree_id` 选择 `RingDemo` / `MiningHaul` /
`PortTransport` / `AgricultureRoute`。Planner 先注册 XML 中的所有树，再按 ID 创建指定树；
`front_dist` 和 `action` 在场景树与共享子树之间显式映射。未知 ID、加载或结构错误会拒绝节点初始化，不会带着半初始化
树运行。这些仍属于基础行为策略；装卸点、会车、作业行与地头转弯等业务节点尚未接入
Runtime。

`behavior_profile` **不会切换地图**；它只选择行为策略，`auto` 使用当前
场景的默认 Profile。`scenario` 才选择地图、初始位姿、Mission route、障碍物
和场景约束。当前已支持 `ring_demo` 与最小 `mining_haul`；港口和农业
场景仍待实现。`bt_tree_id` 仅作为 `behavior_profile` 的兼容别名保留。

场景地基已开始迁移：`ScenarioDefinition` 和 `RingScenarioDefinition` 不依赖 ROS，
环形默认位姿、闭合路线、FollowRoute Mission、车辆约束及默认
`RingDemo` Profile 已由该合同提供。`RingMap` 暂时仍负责 RViz Marker 和环道
静态障碍，后续再迁到完整 Scenario Adapter。

## ROS 2 接口收敛目标

ROS 命名空间由 launch 参数传入，例如 `robot_namespace:=car01`；节点内使用相对名称，禁止硬编码 `/sdc`。

| 方向 | 相对接口 | 语义 |
|---|---|---|
| input | `localization/odometry` | 位姿、速度、时间戳 |
| input | `perception/obstacles` | 障碍物集合 |
| action | `mission/execute` | 明确任务与生命周期 |
| input | `safety/emergency_stop` | 显式 set 语义 |
| output | `planning/trajectory` | 统一稠密轨迹 |
| output | `control/command` | 车辆控制命令 |
| output | `runtime/status` | 状态、任务、健康度 |
| output | `runtime/faults` | 活跃故障集合 |
| output | `runtime/metrics` | 循环耗时、目标频率、轨迹点数、故障数 |
| service | `runtime/health` | 当前 Runtime 健康检查 |
| service | `runtime/acknowledge_recovery` | 满足恢复前置条件后确认解除软件安全锁存 |

以上正式接口已定义为 ROSIDL：`Trajectory`、`ControlCommand`、
`RuntimeStatus`、`FaultArray` 和 `ExecuteMission` Action。当前节点已发布统一轨迹、安全控制、状态、故障和指标输出，并提供
`mission/execute` Action Server，支持忙时拒绝、显式抢占、取消、进度反馈和终态结果。
环形教学任务仍在启动时创建，外部任务需要 `allow_preempt=true` 才能抢占它。
`mission_id` 同时作为幂等键，最近 256 个 ID 不会被重复执行；同一运行周期发布的
typed 状态、轨迹、控制、故障和指标共享单调递增 `sequence`、时间戳和明确 frame。

当前 `ring_road_sim` 的 `sdc/*` 是 namespace 下的兼容教学接口，不是最终 Runtime API。迁移状态见 [TODO.md](TODO.md)。

QoS 已按数据语义分级：Odometry/LIDAR 使用 sensor best-effort，任务控制和轨迹使用
reliable，RuntimeStatus/Fault 和静态地图使用 reliable + transient-local，避免新订阅者
长期看不到最近健康状态或地图。

基础可观测性通过结构化 Status/Fault 消息、`runtime/metrics` 和
`runtime/health` 提供。循环耗时为 ROS 节点一次完整 timer callback 的壁钟耗时，
可用于发现超周期；后续可由 Gateway 转换成 Prometheus 指标。

迁移期已额外提供 `sdc/runtime_state`、`sdc/fault_count`、
`sdc/mission_state`、`sdc/mission_progress` 和 `sdc/emergency_stop`。
状态枚举暂时以整数发布，仅作为 `autonomy_msgs` 落地前的兼容接口。
急停使用明确 set 语义：发布 `true` 进入 ESTOP，
发布 `false` 清除请求；恢复运动仍需满足定位、状态 freshness 和规划健康条件。

## 构建与默认教学场景

```bash
source /opt/ros/jazzy/setup.bash   # 或 humble
colcon build --packages-select self_driving_car_demo
source install/setup.bash
ros2 launch self_driving_car_demo ring_road.launch.py
# 多车/隔离接口：话题位于 /car01/...
ros2 launch self_driving_car_demo ring_road.launch.py robot_namespace:=car01
# 真正切换到最小矿区运输场景（自动选 MiningHaul Profile）
ros2 launch self_driving_car_demo ring_road.launch.py scenario:=mining_haul
# 场景和行为策略可交叉组合做测试
ros2 launch self_driving_car_demo ring_road.launch.py \
  scenario:=mining_haul behavior_profile:=RingDemo
```

本包生成 ROSIDL 的 C/C++ typesupport，因此 CMake 项目同时启用 `C` 和 `CXX`
语言。如果在修正前已经生成过旧 CMake 缓存，请使用下面的一次性重配置：

```bash
colcon build --packages-select self_driving_car_demo --cmake-clean-cache
```

`launch/`、`rviz/`、`config/` 和 `docs/` 会保留目录层级安装到 package share。
如果已经使用过旧的平铺安装规则，需删除该包的 `build/` 和 `install/`
产物后重建，否则 ament index 仍可能指向不完整的旧安装空间。

运行参数集中在 `config/runtime.yaml`，当前支持 `robot_id`、`world_frame`、
`base_frame`、`update_rate_hz`、`lidar_rate_hz`、`initial_mode`、`bt_xml`、`scenario` 和
`behavior_profile`。launch 可通过
`params_file` 替换整套配置，并可单独覆盖 `robot_id`。空标识以及超出支持范围
的循环频率会在节点初始化阶段被拒绝。

同一 YAML 还集中配置 `planner` 的点间距/前视/障碍余量、Pure Pursuit 的轴距/
前视/最大转角、速度规划的横向与纵向约束，以及 Safety 状态超时。ROS Adapter
完成参数合法性检查后，通过 Runtime 的可替换组件接口注入，不让核心层读取 ROS 参数。

默认 Demo 使用 `ReferencePathPlanner + VelocityPlanner + PurePursuitController`、随机障碍、Ackermann 模型和 RViz 可视化。旧 Lattice/EM、Stanley/LQR/MPC/AutoDriver 代码仅作历史教学对照，不进入默认构建，也不暴露直接切换内部实现的正式 ROS 控制接口。

## 目录（迁移态）

```text
include/{domain,runtime,mission,safety,planning}/  新 Runtime API
src/{runtime,mission,safety,planning}/             新 Runtime 实现
include|src/control/trajectory_controller.*        统一轨迹控制接口
include|src/simulation/simulation_engine.*         ROS-free 仿真执行器
include|src/vehicle/                               仿真/真实底盘适配边界
config/behavior_trees/                             场景行为树
include|src/{sim,behavior_tree,...}/               现有算法/教学适配器
src/ros2/                                          ROS 节点（待瘦身）
```

## 设计约束

1. 核心层不得 include ROS 消息。
2. planner 不得知道圆环、矿区或农田；场景只提供路径和约束。
3. 所有 planner 输出同一种 `Trajectory`，所有 controller 消费它。
4. Mission 是外部控制面；算法切换只保留为调试能力。
5. Safety 是命令发布前的最后仲裁者，默认失效安全。
6. 仿真与实车复用同一 Runtime，仅 adapter 不同。

## 核心库与测试

构建系统已将 `sdc_runtime_core` 与依赖 ROS visualization 的环道教学
adapter `sdc_core` 分离。前者包含 domain、mission、planning、velocity、control、
safety、vehicle model 与 SimulationEngine，可独立进行纯 C++ 单元测试。
带圆环假设的 legacy planner/controller/AutoDriver 不属于任一正式构建目标。

`test/runtime_core_test.cpp` 当前覆盖 Mission 校验/抢占/超时、非环形折线
稠密规划、动态障碍局部绕行、正常非闭合路线闭环、安全停车、受控安全恢复、
Stop/Navigate Mission、定位丢失、规划失败、感知断流和急停。代码已加入构建配置，
但按协作约定尚未执行，需由用户验证：

```bash
colcon test --packages-select self_driving_car_demo
colcon test-result --verbose
```

`.github/workflows/ci.yml` 提供 ROS 2 Jazzy 容器中的 rosdep、构建和测试门禁；
尚未在本地执行，也尚未加入 clang-format/clang-tidy 门禁。

## Record / Replay

`RuntimeRecorder` 可记录每周期车辆状态、障碍物、Runtime 状态、安全控制和完整
稠密轨迹，并以带魔数的 `SDC_REPLAY_V1` 高精度文本格式保存/加载。
`replayFrame()` 将记录输入重新送入任意 `VehicleRuntime`，用于离线调试和确定性回归。
SimulationEngine 可通过 `enableRecording(true)` 开启内存记录。Mission 定义需在回放开始
前由测试或工具显式安装，ROS 录制控制接口和自动误差阈值仍待实现。

`SimulationEngine` 负责仿真时间、Ackermann 车辆模型、手动输入和 Runtime 控制指令执行。`ring_road_sim_node` 已通过该边界运行，不再直接持有 `VehicleRuntime`、车辆模型或旧 `AutoDriver`。RViz Marker/LIDAR 生成仍在节点内，下一步继续抽成独立 Visualization Adapter。

仿真引擎提供定位和感知可用性故障注入，用于可重复验证 watchdog 与安全停车。
接口冲突的旧 `car_controller_node.cpp` 已从构建和安装目标移除，源码暂留作历史参考，
不会再与正式 Runtime 同时发布控制结果。
