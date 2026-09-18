# Autonomy Runtime 实施路线

标记：`[x]` 已完成，`[~]` 已建边界但尚未接入默认环形闭环，`[ ]` 未开始。

## M1 — Core contract（本轮）

- [x] ROS-free `VehicleState / Mission / Fault / ControlCommand`
- [x] 统一稠密 `Trajectory<Waypoint>`
- [x] 通用 `Planner` 接口及 `ReferencePathPlanner` 基线
- [x] `MissionManager` 生命周期
- [x] `FaultManager + SafetyManager` 与最终指令覆盖
- [x] `VehicleRuntime::step()` 编排骨架
- [x] 行为树支持外部 XML，提供四类 Behavior Profile 入口
- [x] 独立 `sdc_runtime_core` 无 ROS 构建目标
- [~] 已增加 Mission/Planner/Safety/Runtime 单元测试，待用户编译执行确认

验收：任意折线路径能生成约 0.25 m 间距轨迹；状态超时、定位丢失、规划失败和急停均输出停车命令。

## M2 — 接管默认闭环

- [~] `ring_road_sim_node` 已接入 ROS-free `SimulationEngine`；Visualization Adapter 待拆分
- [x] 环形道路生成通用 `Mission.route`
- [x] 默认闭环切换至 `VehicleRuntime`；legacy 算法源码隔离保留，不再进入正式控制面
- [x] 旧 Lattice/EM/AutoDriver 与环道几何类型解耦，并从默认构建隔离；不将其冒充为通用 Planner
- [x] controller 统一消费 `Trajectory`；支持注入 `Controller` 实现
- [x] 几何路径与速度规划分层，增加曲率/加减速约束
- [x] 建立 `VehicleInterface`，仿真车辆通过 `SimulatedVehicle` adapter 接入
- [x] 接口冲突的旧 `car_controller_node` 已从构建与安装目标隔离，源码仅保留参考

验收：环形 Demo 不退化；相同 Runtime 使用非闭合折线路径也能工作；Node 只负责 callback/timer/converter。

## M3 — ROS 接口与高可用

- [~] ExecuteMission Action Server 与 Trajectory/Control/Status/Fault 输出已接入；ROSIDL 已同时启用 C/C++ 生成语言，package share 已保留 config/rviz/launch 目录，待用户清理旧安装空间后重建及端到端验证
- [x] topic 相对命名，namespace/robot_id/update_rate/frame/YAML/bt_xml 已参数化
- [~] sensor/command/status/static QoS 已分级；Lifecycle Node 待实现
- [~] typed 输出已携带统一 sequence/stamp/frame，Mission 校验 frame 并按 mission_id 去重；输入时间窗与多源 sequence 策略待真实 Adapter
- [~] 定位/感知 freshness、车辆状态有限值、规划及控制输出 watchdog 已进入 Runtime；真实底盘反馈 watchdog 待 Vehicle Adapter
- [x] Planner/Controller/Velocity/Safety 核心参数已映射 YAML 并执行初始化合法性校验；Safety 核心库也独立保护配置不变式
- [x] RuntimeStatus/Fault/Metrics 结构化输出与 health service
- [x] Domain→ROS typed message 转换已抽离为独立 `sdc_ros_adapter`

验收：同域启动 `car01`、`car02` 不冲突；上游断流后在限定时间停车并上报故障。

## M4 — Mission / Behavior / Safety 完整化

- [~] NavigateTo/FollowRoute/Stop 已接入 ROS Action，待用户端到端验证；其余任务能力插件化待实现
- [x] Mission 参数校验、preempt、timeout、进度和结果原因
- [~] BT 已通过 BehaviorManager 接入 Runtime，单 XML 复用参数化子树，`bt_tree_id` 显式选 Behavior Profile 且黑板端口完整映射；装卸/会车/地头业务节点待实现
- [x] Fault policy 的 report/degrade/stop/estop 已映射 YAML；STOP/ESTOP 故障锁存，仅在输入连续健康、车辆静止并调用 recovery acknowledgement 后解除
- [~] SimulationEngine 已支持定位/感知故障注入并覆盖测试；硬件急停与真实 CAN Vehicle Adapter 待实现

## M5 — 真实场景适配层

> 当前 `bt_tree_id` 只选择 Behavior Profile；地图、Mission route、障碍物生成和车辆模型仍是环形教学场景。

- [~] 已建立 ROS-free `ScenarioDefinition` 及合同校验，统一初始位姿、参考路线、静态障碍、默认 Mission、车辆约束与 Behavior Profile；文件 `ScenarioLoader` 待场景资产格式确定后实现
- [x] 已将 `scenario` 与 `behavior_profile` 分离参数化；`auto` 选场景默认 Profile，也允许显式交叉组合
- [~] 已实现 ROS-free `RingScenarioDefinition`，默认位姿/路线/Mission 已从 ROS 节点迁出并由环形 adapter 消费；地图 Marker 与静态障碍仍在 `RingMap`
- [~] 已实现 `mining_haul` 最小可运行场景：LOAD→DUMP 非闭合高程运输路线、LOAD/DUMP 可视化、确定性路侧障碍、低速 Mission 与 MiningHaul Profile；返回、重载/空载状态切换待实现
- [ ] 环形基准验收：无障碍稳定跟踪、单障碍绕行、连续障碍、规划失败停车、闭环任务五项均通过
- [x] 环形基准改为双车道（总宽 6 m），固定中心线障碍物用于可重复验证变道/绕障，支持 `simulation.fixed_ring_obstacles` 开关
- [x] 环形默认障碍物改为随机模式，障碍物状态每 3 秒更新；固定模式仅用于回归测试
- [~] 环形场景已切换为 `RingLanePlanner`：KEEP_LEFT/CHANGE_RIGHT/KEEP_RIGHT/CHANGE_LEFT 状态与连续 Frenet 变道轨迹；待用户编译验证和补齐目标车道占用/连续障碍测试
- [x] RingLanePlanner 对膨胀障碍物执行硬碰撞否决；无无碰撞轨迹时返回 planning failure，由 SafetyManager 停车
- [ ] 矿区业务闭环：`LOAD → 装载确认 → HAUL → DUMP → 卸载确认 → RETURN/结束`，增加载荷状态、装卸点停靠和任务事件
- [ ] 实现 `port_transport` 最小场景：堆场→路口→岸桥交接点，支持路权/停车线和精准停靠任务
- [ ] 实现 `agriculture_route` 最小场景：作业行路线、地头转弯和作业机具状态，保持人员/作物安全边界
- [ ] 港口扩展：堆场/闸口/岸桥/充电位、交叉口路权、倒车/精准对接、作业区限速
- [ ] 农业扩展：作业行生成、地头 U 型/回字转弯、机具状态、行间偏差和人员/牲畜安全区
- [ ] 为装卸、会车/路权、精准停靠、作业行/地头转弯增加可复用 BT 业务子树，而非把场景逻辑写入 Runtime
- [~] `ring_demo` / `mining_haul` 已可通过同一 launch 的 `scenario` 参数一键切换，矿区已有确定性路线/障碍；独立 YAML 资产及其他场景待实现
- [~] `ring_demo` / `mining_haul` 已可通过同一 launch 的 `scenario` 参数一键切换，矿区已有高程路线、确定性障碍和三维 RViz/轨迹显示；独立 YAML 资产及其他场景待实现
- [ ] 增加场景合同测试：路线有效、初始位姿可行、Behavior Profile 可加载、Mission 可达成、故障仍由 Safety 最终仲裁

验收：`scenario:=ring_demo|mining_haul|port_transport|agriculture_route` 会真正改变地图、路线、任务和环境；`behavior_profile` 只改变行为/安全策略。四个场景共用同一 `VehicleRuntime`、`Trajectory`、`VehicleInterface` 和 ROS 合同。

## M6 — 验证与教学

- [~] core unit 与版本化 Runtime record/replay 已建立；文件回放回归阈值和 ROS replay CLI 待完成
- [x] 已建立正常非闭合路线闭环、动态障碍规划、定位丢失、规划失败、感知断流和急停回归测试；待用户编译执行确认
- [x] 架构边界、Runtime 时序图、接口契约、三类场景扩展与故障指南
- [~] GitHub Actions 已覆盖 rosdep/build/unit test；format/static analysis/integration 待补

## 暂不做

- 不继续堆 planner；先完成现有算法接口化。
- 不引入 Kafka、Service Mesh 或车端微服务化。
- 不让云端直接设置 controller/planner 作为正式任务接口。
- 不用软件 Safety 替代底盘硬件安全链路。
- [ ] 将现有 Lattice/EM/Stanley/LQR/MPC 通过统一 adapter 接入 `VehicleRuntime`，提供 `planner.type` / `controller.type` 参数；当前它们仍属于旧 AutoDriver 链路

## 下一阶段总原则

先完成一条可复用、可验证的自动驾驶闭环，而不是同时堆四个 Demo：

```text
场景路线 → Mission → Behavior → Planner → dense Trajectory
→ Velocity Planner → Controller → Safety → VehicleInterface
```

环形道路验证 VehicleRuntime 基础闭环；矿区验证 `LOAD→DUMP` 装运流程；农业验证
作业行与地头转弯；港口验证路权、精准停靠和交接作业。场景只提供路线、作业点、
约束和业务 BT，不复制 Runtime、规划器或控制器。

## M0 — 目录与边界收敛

- [ ] 将 `src/ros/` 明确为 `ros_adapter/`（保留兼容路径），避免与 `src/ros2/` 造成两套 ROS 层的误解
- [ ] 将 `ring_road_sim_node` 拆为 Runtime Node、Visualization Adapter、Sensor Adapter
- [ ] 将仍带环道假设的 legacy 代码标记并隔离，禁止新 Runtime 直接依赖
- [ ] 统一只从 `autonomy.launch.py` 启动，清理文档中的 `ring_road` 领域命名

## M7 — 自动驾驶必备能力补齐

- [ ] 定位：Odometry、frame/时间戳校验、定位丢失降级和恢复
- [ ] 感知：障碍物集合、动态障碍速度/预测、传感器 freshness
- [ ] 地图与路线：参考路径、局部路径、作业行语义、速度区和禁行区
- [ ] 规划：全局路线、局部避障、轨迹平滑、曲率约束、可行性检查和失败原因
- [ ] 控制：横向跟踪、纵向速度、转角/转角速度限幅、倒车和停车控制
- [ ] 车辆模型：Ackermann 运动学、速度/加减速约束、真实底盘反馈接口
- [ ] 任务行为：装卸、停靠、会车、路权、地头转弯等业务动作
- [ ] 安全：急停、看门狗、碰撞约束、速度上限、故障锁存、恢复确认和硬件急停接口
- [ ] 测试：算法单测、契约测试、场景测试、故障注入、E2E 和确定性回放
