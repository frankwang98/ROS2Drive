/**
 * ring_road_sim_node.cpp — 环形道路自动驾驶仿真节点
 *
 * 单张「环形道路」地图，在基础环形基础上增加了道路复杂性（S 形绕桩、
 * 窄门、减速带、多车道标线等），并支持 **自动 / 手动** 两种驾驶模式：
 *   - 自动驾驶：Mission → Planner → Velocity → Controller → Safety → Vehicle
 *   - 手动驾驶：通过 WASD 键盘直接控制（W/S 油门/倒车，A/D 转向）
 *
 * 功能：
 *   1. RViz2 Marker 绘制环形道路（含复杂路况）
 *   2. 统一稠密轨迹与局部避障结果可视化
 *   3. VehicleRuntime 规划、速度、控制与安全闭环
 *   4. 自动/手动驾驶模式切换（HUD 面板或话题）
 *   5. 随机动态障碍物 + 静态复杂路况避障
 *   6. TF / LIDAR / 状态文本等可视化
 *
 * 话题：
 *   订阅：
 *     /sdc/set_mode       (Int32)  0=自动驾驶 1=手动驾驶
 *     /sdc/manual_cmd     (Twist)  手动指令（linear.x=油门 -1..1，angular.z=转向 -1..1）
 *     /sdc/start          (Bool)   开始行驶（自动）
 *     /sdc/pause          (Bool)   暂停/继续行驶
 *     /sdc/reset_car      (Bool)   把车放回环形起点
 *     /sdc/clear_trail    (Bool)   清除轨迹
 *   发布：
 *     /sdc/speed          (Float64) 当前车速
 *     /sdc/action_id      (Float64) 行为ID
 *     /sdc/front_distance (Float64) 前方最近障碍距离
 *     /sdc/obstacle_count (Float64) 动态障碍数量
 *     /sdc/mode           (Int32)   当前驾驶模式
 *     （其余 HUD 话题保持原样）
 */

#include <chrono>
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <cstdint>
#include <deque>
#include <memory>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include "self_driving_car_demo/msg/control_command.hpp"
#include "self_driving_car_demo/msg/fault_array.hpp"
#include "self_driving_car_demo/msg/runtime_status.hpp"
#include "self_driving_car_demo/msg/runtime_metrics.hpp"
#include "self_driving_car_demo/msg/trajectory.hpp"
#include "self_driving_car_demo/action/execute_mission.hpp"

#include "sim/map.hpp"
#include "sim/obstacle_manager.hpp"
#include "planning/reference_path_planner.hpp"
#include "planning/coverage_path_planner.hpp"
#include "planning/ring_lane_planner.hpp"
#include "planning/legacy_planner_adapter.hpp"
#include "control/legacy_controller_adapter.hpp"
#include "simulation/simulation_engine.hpp"
#include "behavior_tree/behavior_tree_behavior.hpp"
#include "ros/runtime_message_converter.hpp"
#include "scenario/ring_scenario.hpp"
#include "scenario/mining_haul_scenario.hpp"
#include "scenario/agriculture_route_scenario.hpp"
#include "scenario/port_transport_scenario.hpp"

using namespace std::chrono_literals;
using namespace sdc;

// ========== 仿真参数 ==========
constexpr double LIDAR_RANGE     = 35.0;   // LIDAR 探测距离（米）
constexpr int    LIDAR_BEAMS     = 360;    // 水平扫描线数
constexpr int    PATH_SAMPLES    = 60;     // 规划路径采样点
constexpr double PATH_LENGTH     = 30.0;   // 规划路径可视长度（米）
constexpr int    TRAIL_MAX_POINTS = 600;   // 行驶轨迹最多保留点数
constexpr double CAR_HEAD_Z      = 2.6;    // 状态文本高度

// ========== 工具函数 ==========
static geometry_msgs::msg::Point make_point(double x, double y, double z = 0.0) {
  geometry_msgs::msg::Point p;
  p.x = x; p.y = y; p.z = z;
  return p;
}

static std::string runtime_state_name(domain::RuntimeState state) {
  switch (state) {
    case domain::RuntimeState::kInit: return "INIT";
    case domain::RuntimeState::kReady: return "READY";
    case domain::RuntimeState::kRunning: return "RUNNING";
    case domain::RuntimeState::kPaused: return "PAUSED";
    case domain::RuntimeState::kDegraded: return "DEGRADED";
    case domain::RuntimeState::kStopped: return "STOPPED";
    case domain::RuntimeState::kFault: return "FAULT";
    case domain::RuntimeState::kEstop: return "ESTOP";
  }
  return "UNKNOWN";
}

static std::string mission_state_name(domain::MissionState state) {
  switch (state) {
    case domain::MissionState::kPending: return "PENDING";
    case domain::MissionState::kActive: return "ACTIVE";
    case domain::MissionState::kPaused: return "PAUSED";
    case domain::MissionState::kSucceeded: return "SUCCEEDED";
    case domain::MissionState::kFailed: return "FAILED";
    case domain::MissionState::kCanceled: return "CANCELED";
  }
  return "UNKNOWN";
}

static domain::FaultAction fault_action_from_string(const std::string& value) {
  if (value == "report") return domain::FaultAction::kReportOnly;
  if (value == "degrade") return domain::FaultAction::kDegrade;
  if (value == "stop") return domain::FaultAction::kStop;
  if (value == "estop") return domain::FaultAction::kEmergencyStop;
  throw std::invalid_argument("unknown safety policy action: " + value);
}

static const char* mission_stage_name(domain::MissionStage stage,
                                      const std::string& scenario_id = "") {
  if (scenario_id == "port_transport") {
    switch (stage) {
      case domain::MissionStage::kTransit: return "YARD_TRANSIT";
      case domain::MissionStage::kLoad: return "GATE_WAIT";
      case domain::MissionStage::kHaul: return "QUAY_TRANSIT";
      case domain::MissionStage::kDump: return "QUAY_DOCK";
      case domain::MissionStage::kReturn: return "RETURN";
    }
  }
  switch (stage) {
    case domain::MissionStage::kLoad: return "LOAD";
    case domain::MissionStage::kHaul: return "HAUL";
    case domain::MissionStage::kDump: return "DUMP";
    case domain::MissionStage::kReturn: return "RETURN";
    default: return "TRANSIT";
  }
}


class RingRoadSimNode : public rclcpp::Node {
public:
  using ExecuteMission = self_driving_car_demo::action::ExecuteMission;
  using MissionGoalHandle = rclcpp_action::ServerGoalHandle<ExecuteMission>;
  RingRoadSimNode()
      : Node("ring_road_sim")
      , map_(create_map(MapType::kRing))
      , simulation_(std::make_unique<planning::ReferencePathPlanner>())
      , sim_time_(0.0)
      , obstacle_manager_(26.0, 9.0, 20240812)
  {
    robot_id_ = declare_parameter<std::string>("robot_id", "car01");
    world_frame_ = declare_parameter<std::string>("world_frame", "world");
    base_frame_ = declare_parameter<std::string>("base_frame", "car_base_link");
    const double update_rate_hz = declare_parameter<double>("update_rate_hz", 20.0);
    const double lidar_rate_hz = declare_parameter<double>("lidar_rate_hz", 10.0);
    fixed_ring_obstacles_ = declare_parameter<bool>("simulation.fixed_ring_obstacles", true);
    auto_recover_safety_ = declare_parameter<bool>("simulation.auto_recover_safety", true);
    const std::string bt_xml = declare_parameter<std::string>("bt_xml", "");
    const std::string scenario_id =
        declare_parameter<std::string>("scenario", "ring_demo");
    const std::string configured_behavior_profile =
        declare_parameter<std::string>("behavior_profile", "auto");
    const std::string controller_type =
        declare_parameter<std::string>("controller.type", "auto");
    const std::string planner_type =
        declare_parameter<std::string>("planner.type", "auto");
    // Compatibility alias. Prefer behavior_profile for new launch files.
    const std::string legacy_bt_tree_id =
        declare_parameter<std::string>("bt_tree_id", "");
    if (scenario_id == "ring_demo") {
      scenario_definition_ = scenario::makeRingScenarioDefinition();
      map_ = create_map(MapType::kRing);
    } else if (scenario_id == "mining_haul") {
      scenario_definition_ = scenario::makeMiningHaulScenarioDefinition();
      map_ = std::make_unique<DefinitionScenarioMap>(scenario_definition_);
    } else if (scenario_id == "agriculture_route") {
      scenario_definition_ = scenario::makeAgricultureRouteScenarioDefinition();
      map_ = std::make_unique<DefinitionScenarioMap>(scenario_definition_);
    } else if (scenario_id == "port_transport") {
      scenario_definition_ = scenario::makePortTransportScenarioDefinition();
      map_ = std::make_unique<DefinitionScenarioMap>(scenario_definition_);
    } else {
      throw std::invalid_argument("unsupported scenario: " + scenario_id);
    }
    const std::string behavior_profile =
        !legacy_bt_tree_id.empty()
            ? legacy_bt_tree_id
            : (configured_behavior_profile == "auto"
                   ? scenario_definition_.default_behavior_profile
                   : configured_behavior_profile);
    planning::ReferencePathPlanner::Config planner_config;
    planner_config.spacing = declare_parameter<double>("planner.spacing", 0.25);
    planner_config.horizon = declare_parameter<double>("planner.horizon", 30.0);
    planner_config.obstacle_margin = declare_parameter<double>("planner.obstacle_margin", 0.8);
    planner_config.smooth_corners = scenario_id == "port_transport";
    control::PurePursuitController::Config controller_config;
    // The current VehicleRuntime consumes a dense reference trajectory, so
    // Pure Pursuit is the integrated controller.  Use a tighter lookahead for
    // the sparse, low-speed mining haul road; it prevents cutting corners.
    const double wheelbase_default = scenario_id == "mining_haul" ? 2.8 :
                                     scenario_id == "agriculture_route" ? 2.2 :
                                     scenario_id == "port_transport" ? 2.7 : 2.0;
    const double lookahead_default = scenario_id == "mining_haul" ? 0.8 :
                                     scenario_id == "agriculture_route" ? 1.8 :
                                     scenario_id == "port_transport" ? 1.2 : 1.5;
    const double lookahead_time_default = scenario_id == "mining_haul" ? 0.5 :
                                          scenario_id == "agriculture_route" ? 1.0 : 1.0;
    controller_config.wheelbase = declare_parameter<double>("controller.wheelbase", wheelbase_default);
    controller_config.minimum_lookahead = declare_parameter<double>("controller.minimum_lookahead", lookahead_default);
    if (scenario_id == "agriculture_route" && controller_config.minimum_lookahead >= 1.5) {
      controller_config.minimum_lookahead = declare_parameter<double>(
          "controller.agriculture_minimum_lookahead", 1.2);
    } else {
      declare_parameter<double>("controller.agriculture_minimum_lookahead", 1.2);
    }
    controller_config.lookahead_time = declare_parameter<double>("controller.lookahead_time", lookahead_time_default);
    const double maximum_steering_default = scenario_id == "agriculture_route" ? 0.80 : 0.55;
    controller_config.maximum_steering = declare_parameter<double>(
        "controller.maximum_steering", maximum_steering_default);
    if (scenario_id == "agriculture_route" && controller_config.maximum_steering <= 0.55) {
      controller_config.maximum_steering = declare_parameter<double>(
          "controller.agriculture_maximum_steering", 0.80);
    } else {
      declare_parameter<double>("controller.agriculture_maximum_steering", 0.80);
    }
    planning::VelocityPlanner::Config velocity_config;
    velocity_config.maximum_lateral_acceleration = declare_parameter<double>("velocity.maximum_lateral_acceleration", 1.2);
    velocity_config.maximum_acceleration = declare_parameter<double>("velocity.maximum_acceleration", 1.0);
    velocity_config.maximum_deceleration = declare_parameter<double>("velocity.maximum_deceleration", 1.5);
    velocity_config.minimum_curve_speed = declare_parameter<double>("velocity.minimum_curve_speed", 0.3);
    safety::SafetyConfig safety_config;
    safety_config.state_timeout_s = declare_parameter<double>("safety.state_timeout", 0.5);
    const int recovery_healthy_cycles =
        declare_parameter<int>("safety.recovery_healthy_cycles", 3);
    safety_config.recovery_healthy_cycles =
        static_cast<unsigned>(recovery_healthy_cycles);
    safety_config.policies[domain::FaultCode::kLocalizationLost] = fault_action_from_string(
        declare_parameter<std::string>("safety.policies.localization_lost", "stop"));
    safety_config.policies[domain::FaultCode::kPlanningFailed] = fault_action_from_string(
        declare_parameter<std::string>("safety.policies.planning_failed", "stop"));
    safety_config.policies[domain::FaultCode::kControlError] = fault_action_from_string(
        declare_parameter<std::string>("safety.policies.control_error", "stop"));
    safety_config.policies[domain::FaultCode::kVehicleError] = fault_action_from_string(
        declare_parameter<std::string>("safety.policies.vehicle_error", "estop"));
    safety_config.policies[domain::FaultCode::kInputTimeout] = fault_action_from_string(
        declare_parameter<std::string>("safety.policies.input_timeout", "stop"));
    safety_config.policies[domain::FaultCode::kPerceptionTimeout] = fault_action_from_string(
        declare_parameter<std::string>("safety.policies.perception_timeout", "stop"));
    safety_config.policies[domain::FaultCode::kEmergencyStop] = fault_action_from_string(
        declare_parameter<std::string>("safety.policies.emergency_stop", "estop"));
    if (robot_id_.empty() || world_frame_.empty() || base_frame_.empty() ||
        behavior_profile.empty())
      throw std::invalid_argument("robot_id, frames and behavior_profile must not be empty");
    if (update_rate_hz < 1.0 || update_rate_hz > 100.0 ||
        lidar_rate_hz < 1.0 || lidar_rate_hz > 50.0)
      throw std::invalid_argument("update_rate_hz or lidar_rate_hz outside supported range");
    if (planner_config.spacing <= 0.0 || planner_config.horizon <= 0.0 ||
        planner_config.obstacle_margin < 0.0 || controller_config.wheelbase <= 0.0 ||
        controller_config.minimum_lookahead <= 0.0 || controller_config.maximum_steering <= 0.0 ||
        velocity_config.maximum_acceleration <= 0.0 || velocity_config.maximum_deceleration <= 0.0 ||
        safety_config.state_timeout_s <= 0.0 || recovery_healthy_cycles <= 0)
      throw std::invalid_argument("planner/controller/velocity/safety parameters must be positive");
    sim_dt_ = 1.0 / update_rate_hz;
    configured_loop_hz_ = update_rate_hz;
    if (!bt_xml.empty()) {
      auto behavior =
          std::make_unique<BehaviorTreeBehavior>(bt_xml, behavior_profile);
      if (!behavior->configurationAccepted())
        throw std::invalid_argument("BehaviorTree configuration rejected: " +
                                    behavior->lastError());
      if (!behavior->initialized() && !behavior->lastError().empty())
        RCLCPP_WARN(get_logger(), "BehaviorTree fallback: %s", behavior->lastError().c_str());
      simulation_.setBehaviorManager(std::move(behavior));
    }
    const std::string selected_planner = planner_type == "auto"
                                             ? (scenario_id == "ring_demo" ? "ring_lane" :
                                                scenario_id == "agriculture_route" ? "coverage" : "reference_path")
                                             : planner_type;
    const std::string selected_controller = controller_type == "auto"
                                                 ? "pure_pursuit"
                                                 : controller_type;
    RCLCPP_INFO(get_logger(), "Planner selected: %s", selected_planner.c_str());
    if (selected_planner == "ring_lane") {
      if (scenario_id != "ring_demo")
        throw std::invalid_argument("ring_lane planner requires scenario:=ring_demo");
      simulation_.setPlanner(std::make_unique<planning::RingLanePlanner>());
    } else if (selected_planner == "coverage") {
      if (scenario_id != "agriculture_route")
        throw std::invalid_argument("coverage planner requires scenario:=agriculture_route");
      planning::CoveragePathPlanner::Config coverage_config;
      coverage_config.spacing = planner_config.spacing;
      coverage_config.horizon = planner_config.horizon;
      coverage_config.obstacle_margin = planner_config.obstacle_margin;
      coverage_config.row_spacing = declare_parameter<double>("coverage.row_spacing", 5.0);
      coverage_config.headland_radius = declare_parameter<double>("coverage.headland_radius", 2.0);
      simulation_.setPlanner(std::make_unique<planning::CoveragePathPlanner>(coverage_config));
    } else if (selected_planner == "reference_path") {
      simulation_.setPlanner(std::make_unique<planning::ReferencePathPlanner>(planner_config));
    } else if (selected_planner == "lattice" || selected_planner == "em") {
      const auto type = selected_planner == "lattice"
                            ? planning::LegacyPlannerAdapter::Type::kLattice
                            : planning::LegacyPlannerAdapter::Type::kEm;
      simulation_.setPlanner(std::make_unique<planning::LegacyPlannerAdapter>(type));
    } else {
      throw std::invalid_argument("unsupported planner.type: " + selected_planner);
    }
    if (selected_controller == "pure_pursuit") {
      simulation_.setController(std::make_unique<control::PurePursuitController>(controller_config));
    } else if (selected_controller == "stanley" || selected_controller == "lqr" || selected_controller == "mpc") {
      const auto type = selected_controller == "stanley"
                            ? control::LegacyControllerAdapter::Type::kStanley
                            : selected_controller == "lqr"
                                  ? control::LegacyControllerAdapter::Type::kLqr
                                  : control::LegacyControllerAdapter::Type::kMpc;
      simulation_.setController(std::make_unique<control::LegacyControllerAdapter>(
          type, controller_config.wheelbase, controller_config.maximum_steering, sim_dt_));
    } else {
      throw std::invalid_argument("unsupported controller.type: " + selected_controller);
    }
    simulation_.setVelocityPlanner(planning::VelocityPlanner(velocity_config));
    simulation_.setSafetyManager(safety::SafetyManager(safety_config));
    auto road_qos = rclcpp::QoS(1).reliable().transient_local();
    auto sensor_qos = rclcpp::SensorDataQoS().keep_last(5);
    auto live_qos = rclcpp::QoS(10).reliable();
    auto status_qos = rclcpp::QoS(1).reliable().transient_local();
    auto command_qos = rclcpp::QoS(10).reliable();
    road_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
        "simulation/markers", road_qos);
    live_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
        "simulation/markers_live", live_qos);
    obstacle_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
        "simulation/obstacles", live_qos);
    plan_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
        "simulation/lattice", live_qos);
    lidar_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
        "sensor/lidar", sensor_qos);

    // HUD 面板状态话题
    speed_pub_    = create_publisher<std_msgs::msg::Float64>("sdc/speed", live_qos);
    action_pub_   = create_publisher<std_msgs::msg::Float64>("sdc/action_id", live_qos);
    distance_pub_ = create_publisher<std_msgs::msg::Float64>("sdc/front_distance", live_qos);
    obstacle_pub2_ = create_publisher<std_msgs::msg::Float64>("sdc/obstacle_count", live_qos);
    mode_pub_     = create_publisher<std_msgs::msg::Int32>("sdc/mode", live_qos);
    odometry_pub_ = create_publisher<nav_msgs::msg::Odometry>("sdc/odometry", sensor_qos);
    runtime_state_pub_ = create_publisher<std_msgs::msg::Int32>("sdc/runtime_state", live_qos);
    fault_count_pub_ = create_publisher<std_msgs::msg::Int32>("sdc/fault_count", live_qos);
    mission_stage_pub_ = create_publisher<std_msgs::msg::String>("sdc/mission_stage", status_qos);
    mission_state_pub_ = create_publisher<std_msgs::msg::Int32>("sdc/mission_state", live_qos);
    mission_progress_pub_ = create_publisher<std_msgs::msg::Float64>("sdc/mission_progress", live_qos);
    trajectory_pub_ = create_publisher<self_driving_car_demo::msg::Trajectory>("planning/trajectory", live_qos);
    control_command_pub_ = create_publisher<self_driving_car_demo::msg::ControlCommand>("control/command", live_qos);
    status_pub_ = create_publisher<self_driving_car_demo::msg::RuntimeStatus>("runtime/status", status_qos);
    faults_pub_ = create_publisher<self_driving_car_demo::msg::FaultArray>("runtime/faults", status_qos);
    metrics_pub_ = create_publisher<self_driving_car_demo::msg::RuntimeMetrics>("runtime/metrics", live_qos);
    health_service_ = create_service<std_srvs::srv::Trigger>(
        "runtime/health",
        [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
               std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
          const auto state = simulation_.runtime().output().state;
          response->success = state != domain::RuntimeState::kFault &&
                              state != domain::RuntimeState::kEstop;
          response->message = runtime_state_name(state);
        });
    recovery_service_ = create_service<std_srvs::srv::Trigger>(
        "runtime/acknowledge_recovery",
        [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
               std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
          response->success = simulation_.acknowledgeSafetyRecovery();
          response->message = response->success
                                  ? "safety recovery acknowledged"
                                  : "recovery not ready: remove the fault and stop the vehicle";
        });
    mission_action_server_ = rclcpp_action::create_server<ExecuteMission>(
        this, "mission/execute",
        std::bind(&RingRoadSimNode::handle_mission_goal, this,
                  std::placeholders::_1, std::placeholders::_2),
        std::bind(&RingRoadSimNode::handle_mission_cancel, this,
                  std::placeholders::_1),
        std::bind(&RingRoadSimNode::handle_mission_accepted, this,
                  std::placeholders::_1));

    // 开始行驶（自动）
    start_sub_ = create_subscription<std_msgs::msg::Bool>(
        "sdc/start", command_qos, [this](const std_msgs::msg::Bool::SharedPtr msg) {
          if (msg->data) {
            paused_ = false;
            driving_ = true;
            simulation_.resume();
            RCLCPP_INFO(get_logger(), "Driving started");
          }
        });
    // 暂停/继续
    pause_sub_ = create_subscription<std_msgs::msg::Bool>(
        "sdc/pause", command_qos, [this](const std_msgs::msg::Bool::SharedPtr msg) {
          paused_ = msg->data;
          if (paused_) simulation_.pause(); else simulation_.resume();
          RCLCPP_INFO(get_logger(), paused_ ? "Simulation paused" : "Simulation resumed");
        });
    // 清除轨迹
    clear_sub_ = create_subscription<std_msgs::msg::Bool>(
        "sdc/clear_trail", command_qos, [this](const std_msgs::msg::Bool::SharedPtr msg) {
          if (msg->data) { trail_.clear(); RCLCPP_INFO(get_logger(), "Vehicle trail cleared"); }
        });
    // 驾驶模式切换（0=自动 1=手动）
    set_mode_sub_ = create_subscription<std_msgs::msg::Int32>(
        "sdc/set_mode", command_qos, [this](const std_msgs::msg::Int32::SharedPtr msg) {
          set_mode(msg->data == 1 ? Mode::kManual : Mode::kAuto);
        });
    // 手动控制指令（WASD 键盘）
    manual_sub_ = create_subscription<geometry_msgs::msg::Twist>(
        "sdc/manual_cmd", command_qos,
        [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
          manual_throttle_ = msg->linear.x;   // W/S：油门（>0 前进，<0 倒车）
          manual_steer_    = msg->angular.z;   // A/D：转向（-1..1）
        });
    // 重置小车到起点
    reset_sub_ = create_subscription<std_msgs::msg::Bool>(
        "sdc/reset_car", command_qos, [this](const std_msgs::msg::Bool::SharedPtr msg) {
          if (msg->data) { reset_car(); }
        });
    estop_sub_ = create_subscription<std_msgs::msg::Bool>(
        "sdc/emergency_stop", rclcpp::QoS(1).reliable(),
        [this](const std_msgs::msg::Bool::SharedPtr msg) {
          simulation_.requestEmergencyStop(msg->data);
          RCLCPP_WARN(get_logger(), "Software emergency stop: %s", msg->data ? "ACTIVE" : "CLEARED");
        });

    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    // 读取启动参数 initial_mode（0=自动 1=手动）
    int initial_mode = declare_parameter<int>("initial_mode", 0);
    set_mode(initial_mode == 1 ? Mode::kManual : Mode::kAuto);

    timer_ = create_wall_timer(
        std::chrono::duration<double>(sim_dt_),
        std::bind(&RingRoadSimNode::simulation_step, this));
    lidar_timer_ = create_wall_timer(
        std::chrono::duration<double>(1.0 / lidar_rate_hz),
        std::bind(&RingRoadSimNode::publish_lidar, this));

    load_scenario();

    RCLCPP_INFO(get_logger(), "Autonomy simulation started: scenario=%s behavior=%s",
                scenario_definition_.id.c_str(), behavior_profile.c_str());
  }

private:
  enum class Mode { kAuto = 0, kManual = 1 };

  rclcpp_action::GoalResponse handle_mission_goal(
      const rclcpp_action::GoalUUID&,
      std::shared_ptr<const ExecuteMission::Goal> goal) {
    if (goal->mission_id.empty() ||
        goal->mission_type > ExecuteMission::Goal::RETURN_HOME)
      return rclcpp_action::GoalResponse::REJECT;
    if (!goal->header.frame_id.empty() && goal->header.frame_id != world_frame_)
      return rclcpp_action::GoalResponse::REJECT;
    if (simulation_.runtime().missions().busy() && !goal->allow_preempt)
      return rclcpp_action::GoalResponse::REJECT;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_mission_cancel(
      const std::shared_ptr<MissionGoalHandle> handle) {
    if (!active_mission_goal_ || handle != active_mission_goal_)
      return rclcpp_action::CancelResponse::REJECT;
    simulation_.cancelMission();
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_mission_accepted(const std::shared_ptr<MissionGoalHandle> handle) {
    const auto goal = handle->get_goal();
    if (active_mission_goal_ && goal->allow_preempt) {
      auto previous_result = std::make_shared<ExecuteMission::Result>();
      previous_result->success = false;
      previous_result->final_state = "CANCELED";
      previous_result->reason = "preempted_by_new_mission";
      active_mission_goal_->abort(previous_result);
      active_mission_goal_.reset();
    }
    domain::Mission mission;
    mission.id = goal->mission_id;
    mission.type = static_cast<domain::MissionType>(goal->mission_type);
    mission.speed_limit = goal->speed_limit;
    mission.goal_tolerance = goal->goal_tolerance;
    mission.timeout_s = goal->timeout_s;
    for (const auto& pose : goal->route)
      mission.route.push_back({pose.x, pose.y, pose.theta});
    if (!simulation_.setMission(std::move(mission), goal->allow_preempt)) {
      auto result = std::make_shared<ExecuteMission::Result>();
      result->success = false;
      result->final_state = "REJECTED";
      result->reason = simulation_.runtime().missions().lastError();
      handle->abort(result);
      return;
    }
    active_mission_goal_ = handle;
  }

  void update_mission_action() {
    if (!active_mission_goal_) return;
    const auto& mission = simulation_.runtime().missions().current();
    if (!mission) return;
    auto feedback = std::make_shared<ExecuteMission::Feedback>();
    feedback->state = mission_state_name(mission->state);
    feedback->progress = mission->progress;
    feedback->reason = mission->result_reason;
    active_mission_goal_->publish_feedback(feedback);
    const bool terminal = mission->state == domain::MissionState::kSucceeded ||
                          mission->state == domain::MissionState::kFailed ||
                          mission->state == domain::MissionState::kCanceled;
    if (!terminal) return;
    auto result = std::make_shared<ExecuteMission::Result>();
    result->success = mission->state == domain::MissionState::kSucceeded;
    result->final_state = mission_state_name(mission->state);
    result->reason = mission->result_reason;
    if (mission->state == domain::MissionState::kSucceeded)
      active_mission_goal_->succeed(result);
    else if (mission->state == domain::MissionState::kCanceled)
      active_mission_goal_->canceled(result);
    else
      active_mission_goal_->abort(result);
    active_mission_goal_.reset();
  }

  // ========== 地图加载 ==========
  void load_scenario() {
    install_default_scenario_mission();
    trail_.clear();
    driving_ = true;
    paused_ = false;
    road_markers_ = map_->build_road_markers();
    extra_markers_ = map_->build_extra_markers();
    add_mission_zone_markers();
    publish_road_markers();
    RCLCPP_INFO(get_logger(), "Scenario loaded: %s", scenario_definition_.id.c_str());
  }

  void set_mode(Mode m) {
    mode_ = m;
    manual_throttle_ = 0.0;
    manual_steer_ = 0.0;
    RCLCPP_INFO(get_logger(), "Driving mode set to: %s",
                (mode_ == Mode::kManual ? "MANUAL (WASD)" : "AUTONOMOUS"));
  }

  // 重置小车到当前场景起点
  void reset_car() {
    install_default_scenario_mission();
    trail_.clear();
    manual_throttle_ = 0.0;
    manual_steer_ = 0.0;
    RCLCPP_INFO(get_logger(), "Vehicle reset to scenario start: %s",
                scenario_definition_.id.c_str());
  }

  void install_default_scenario_mission() {
    const auto& pose = scenario_definition_.initial_pose;
    simulation_.reset(pose.x, pose.y, pose.yaw);
    auto mission = scenario_definition_.default_mission;
    mission.id += "-" + std::to_string(++scenario_mission_sequence_);
    if (!simulation_.setMission(std::move(mission)))
      throw std::runtime_error("failed to install default scenario mission");
  }

  // ========== 仿真主循环 ==========
  void simulation_step() {
    const auto loop_started = std::chrono::steady_clock::now();
    if (driving_ && !paused_) {
      // 动态随机障碍物
      if (scenario_definition_.id == "ring_demo" && !fixed_ring_obstacles_) {
        dynamic_obstacle_update_accum_ += sim_dt_;
        if (dynamic_obstacle_update_accum_ >= 3.0) {
          obstacle_manager_.update(dynamic_obstacle_update_accum_);
          dynamic_obstacle_update_accum_ = 0.0;
        }
      }
      if (mode_ == Mode::kManual) {
        // 手动驾驶：WASD 直接控制
        double throttle = manual_throttle_;
        double steer = manual_steer_;
        // 当没有任何按键时，松开油门即减速至 0（回归中性）
        last_action_ = Action::kCruise;
        simulation_.manualStep(throttle, steer, sim_dt_);
        last_front_dist_ = driver_front_dist();
      } else {
        std::vector<domain::Obstacle> runtime_obstacles;
        int obstacle_id = 0;
        // The ring benchmark deliberately excludes legacy slalom, narrow-gate
        // and wall pseudo-obstacles.  They were authored for the old single-
        // lane demo and place an inflated wall inside the new left-lane start.
        // Only the explicit fixed lane blockers belong to this benchmark.
        if (scenario_definition_.id != "ring_demo") {
          for (const auto& ob : map_->to_obstacles())
            runtime_obstacles.push_back({"static-" + std::to_string(obstacle_id++),
                                         {ob.position.x, ob.position.y, 0.0}, ob.radius, false});
        }
        if (scenario_definition_.id == "ring_demo" && fixed_ring_obstacles_) {
          constexpr double ring_radius = 24.5;
          // Fixed center-lane obstacles force a reproducible lane-change test.
          for (const auto& spec : std::array<std::pair<double, double>, 3>{{
                   {0.85, 0.0}, {3.00, 0.0}, {5.15, 0.0}}}) {
            const double r = ring_radius + spec.second;
            runtime_obstacles.push_back({"ring-fixed-" + std::to_string(obstacle_id++),
                                         {r * std::cos(spec.first), r * std::sin(spec.first), 0.0},
                                         0.8, false});
          }
        }
        // The obstacle manager belongs to the ring-road benchmark.  Do not
        // leak its generated objects into the obstacle-free mining baseline.
        if (scenario_definition_.id == "ring_demo") {
          for (const auto& ob : obstacle_manager_.obstacles())
            runtime_obstacles.push_back({"dynamic-" + std::to_string(obstacle_id++),
                                         {ob.x, ob.y, 0.0}, ob.radius, true});
        }
        simulation_.setObstacles(std::move(runtime_obstacles));
        const auto output = simulation_.step(sim_dt_);
        if (auto_recover_safety_ && simulation_.runtime().safetyRecoveryReady())
          simulation_.acknowledgeSafetyRecovery();
        last_planning_success_ = output.planning.success;
        last_planning_reason_ = output.planning.reason;
        last_safety_action_ = static_cast<int>(simulation_.runtime().safety().lastAction());
        last_front_dist_ = driver_front_dist();
        last_action_ = output.control.target_speed < 0.05 ? Action::kStop :
                       output.control.target_speed < 1.0 ? Action::kBrake : Action::kCruise;
        if (simulation_.runtime().missions().current()) {
          std_msgs::msg::String stage_message;
          stage_message.data = mission_stage_name(
              simulation_.runtime().missions().current()->stage, scenario_definition_.id);
          mission_stage_pub_->publish(stage_message);
        }
      }

      // 记录轨迹
      sim_time_ += sim_dt_;
      const double vehicle_z = scenario_height_at(simulation_.vehicle().x(),
                                                  simulation_.vehicle().y());
      trail_.push_back(make_point(simulation_.vehicle().x(), simulation_.vehicle().y(), vehicle_z + 0.05));
      if (trail_.size() > TRAIL_MAX_POINTS) trail_.pop_front();

      if (sim_time_ + 1e-9 >= next_status_log_s_) {
        RCLCPP_INFO(get_logger(),
                    "[%.1fs] mode=%-10s stage=%-7s speed=%.2f m/s front=%.2f m action=%-10s plan=%s safety=%d reason=%s",
                    sim_time_, mode_ == Mode::kManual ? "MANUAL" : "AUTONOMOUS",
                    simulation_.runtime().missions().current()
                        ? mission_stage_name(simulation_.runtime().missions().current()->stage,
                                             scenario_definition_.id) : "NONE",
                    simulation_.vehicle().speed(), last_front_dist_, action_name(last_action_),
                    last_planning_success_ ? "OK" : "FAILED", last_safety_action_,
                    last_planning_reason_.c_str());
        do {
          next_status_log_s_ += 1.0;
        } while (next_status_log_s_ <= sim_time_);
      }
    }

    update_mission_action();
    publish_status();
    publish_car_marker();
    publish_obstacles_marker();
    publish_lattice_marker();
    publish_car_tf();

    road_resend_accum_ += sim_dt_;
    if (road_resend_accum_ >= 2.0) {
      road_resend_accum_ = 0.0;
      publish_road_markers();
    }
    const auto loop_finished = std::chrono::steady_clock::now();
    last_loop_duration_ms_ = std::chrono::duration<double, std::milli>(
        loop_finished - loop_started).count();
  }

  // 手动模式下计算前方障碍距离（含静态复杂路况 + 动态障碍）
  double driver_front_dist() {
    std::vector<Obstacle> obstacles;
    if (scenario_definition_.id == "ring_demo" && fixed_ring_obstacles_) {
      constexpr double ring_radius = 24.5;
      for (const double angle : {0.85, 3.00, 5.15})
        obstacles.push_back(Obstacle{{ring_radius * std::cos(angle),
                                      ring_radius * std::sin(angle)}, 0.8});
    } else if (scenario_definition_.id != "ring_demo") {
      obstacles = map_->to_obstacles();
    }
    for (const auto& ob : obstacle_manager_.obstacles())
      obstacles.push_back(Obstacle{Vec2{ob.x, ob.y}, ob.radius});
    double cx = simulation_.vehicle().x(), cy = simulation_.vehicle().y(), cyaw = simulation_.vehicle().yaw();
    const double fx = std::cos(cyaw), fy = std::sin(cyaw);
    double min_d = LIDAR_RANGE;
    for (const auto& ob : obstacles) {
      double dx = ob.position.x - cx, dy = ob.position.y - cy;
      double dist = std::hypot(dx, dy) - ob.radius;
      if (dist < 0) dist = 0.0;
      if (dist < min_d && (dx * fx + dy * fy) > 0.2) min_d = dist;
    }
    return min_d;
  }

  double scenario_height_at(double x, double y) const {
    const auto& route = scenario_definition_.reference_route;
    if (route.empty()) return 0.0;
    double best_distance = std::numeric_limits<double>::max();
    double best_z = route.front().z;
    for (std::size_t i = 1; i < route.size(); ++i) {
      const auto& a = route[i - 1];
      const auto& b = route[i];
      const double dx = b.x - a.x;
      const double dy = b.y - a.y;
      const double length_sq = dx * dx + dy * dy;
      const double t = length_sq > 1e-9
                           ? std::clamp(((x - a.x) * dx + (y - a.y) * dy) /
                                             length_sq,
                                         0.0, 1.0)
                           : 0.0;
      const double px = a.x + t * dx;
      const double py = a.y + t * dy;
      const double distance = std::hypot(x - px, y - py);
      if (distance < best_distance) {
        best_distance = distance;
        best_z = a.z + t * (b.z - a.z);
      }
    }
    return best_z;
  }

  // ========== 状态话题发布 ==========
  void publish_status() {
    auto f = [](double v) { auto m = std_msgs::msg::Float64(); m.data = v; return m; };
    speed_pub_->publish(f(simulation_.vehicle().speed()));
    action_pub_->publish(f(static_cast<double>(static_cast<int>(last_action_))));
    distance_pub_->publish(f(last_front_dist_));
    obstacle_pub2_->publish(f(static_cast<double>(obstacle_manager_.size())));

    std_msgs::msg::Int32 mode;
    mode.data = (mode_ == Mode::kManual) ? 1 : 0;
    mode_pub_->publish(mode);

    nav_msgs::msg::Odometry odom;
    odom.header.stamp = now();
    odom.header.frame_id = world_frame_;
    odom.child_frame_id = base_frame_;
    const double vehicle_z = scenario_height_at(simulation_.vehicle().x(), simulation_.vehicle().y());
    odom.pose.pose.position.x = simulation_.vehicle().x();
    odom.pose.pose.position.y = simulation_.vehicle().y();
    odom.pose.pose.position.z = vehicle_z;
    const double yaw = simulation_.vehicle().yaw();
    odom.pose.pose.orientation.z = std::sin(yaw / 2.0);
    odom.pose.pose.orientation.w = std::cos(yaw / 2.0);
    odom.twist.twist.linear.x = simulation_.vehicle().speed();
    odom.twist.twist.angular.z = simulation_.vehicle().speed() * std::tan(simulation_.vehicle().steer()) /
                                simulation_.vehicle().wheelbase();
    odometry_pub_->publish(odom);
    std_msgs::msg::Int32 runtime_state;
    runtime_state.data = static_cast<int>(simulation_.runtime().output().state);
    runtime_state_pub_->publish(runtime_state);
    std_msgs::msg::Int32 fault_count;
    fault_count.data = static_cast<int>(simulation_.runtime().faults().faults().size());
    fault_count_pub_->publish(fault_count);
    std_msgs::msg::Int32 mission_state;
    std_msgs::msg::Float64 mission_progress;
    const auto& mission = simulation_.runtime().missions().current();
    mission_state.data = mission ? static_cast<int>(mission->state) : -1;
    mission_progress.data = mission ? mission->progress : 0.0;
    mission_state_pub_->publish(mission_state);
    mission_progress_pub_->publish(mission_progress);

    const auto stamp = now();
    const uint64_t sequence = ++message_sequence_;
    ros::MessageContext context{robot_id_, world_frame_, base_frame_, stamp, sequence};
    status_pub_->publish(ros::RuntimeMessageConverter::status(
        simulation_.runtime(), context, true));
    trajectory_pub_->publish(ros::RuntimeMessageConverter::trajectory(
        simulation_.runtime(), context));
    control_command_pub_->publish(ros::RuntimeMessageConverter::control(
        simulation_.runtime(), context));
    faults_pub_->publish(ros::RuntimeMessageConverter::faults(
        simulation_.runtime(), context));
    metrics_pub_->publish(ros::RuntimeMessageConverter::metrics(
        simulation_.runtime(), context, last_loop_duration_ms_,
        configured_loop_hz_));
  }

  // ========== 障碍物标记（动态随机障碍 + 静态复杂路况） ==========
  void publish_obstacles_marker() {
    visualization_msgs::msg::MarkerArray ma;
    int id = 0;
    // Dynamic obstacles are part of the ring benchmark only.  Keep the
    // mining baseline visually and semantically obstacle-free.
    if (scenario_definition_.id == "ring_demo") {
      for (const auto& ob : obstacle_manager_.obstacles()) {
        visualization_msgs::msg::Marker m;
        m.header.frame_id = world_frame_; m.header.stamp = now();
        m.ns = "obstacles"; m.id = id++;
        m.type = visualization_msgs::msg::Marker::CUBE; m.action = m.ADD;
        m.pose.position.x = ob.x; m.pose.position.y = ob.y; m.pose.position.z = 0.5;
        m.pose.orientation.w = 1.0;
        double sz = 2.0 * ob.radius;
        m.scale.x = sz; m.scale.y = sz; m.scale.z = 1.0;
        m.color.r = 0.95f; m.color.g = 0.25f; m.color.b = 0.1f; m.color.a = 0.95f;
        ma.markers.push_back(m);
      }
    }
    if (scenario_definition_.id == "ring_demo" && fixed_ring_obstacles_) {
      constexpr double ring_radius = 24.5;
      int fixed_id = 100;
      for (const double angle : {0.85, 3.00, 5.15}) {
        visualization_msgs::msg::Marker m;
        m.header.frame_id = world_frame_; m.header.stamp = now();
        m.ns = "fixed_ring_obstacles"; m.id = fixed_id++;
        m.type = visualization_msgs::msg::Marker::CYLINDER; m.action = m.ADD;
        m.pose.position.x = ring_radius * std::cos(angle);
        m.pose.position.y = ring_radius * std::sin(angle);
        m.pose.position.z = 0.5; m.pose.orientation.w = 1.0;
        m.scale.x = 1.6; m.scale.y = 1.6; m.scale.z = 1.0;
        m.color.r = 0.95f; m.color.g = 0.25f; m.color.b = 0.1f; m.color.a = 0.95f;
        ma.markers.push_back(m);
      }
    }
    // 静态路况障碍（绕桩锥桶/窄门/路障，黄色/红色）
    // 这里仅绘制锥桶等静态特征，简化处理：复用 map 静态障碍中的桩桶标记
    // （桩桶等已通过 build_road_markers 的可视化体现，此处不重复绘制）
    obstacle_pub_->publish(ma);
  }

  // ========== 局部规划候选路径可视化（自动模式） ==========
  void publish_lattice_marker() {
    visualization_msgs::msg::MarkerArray ma;
    visualization_msgs::msg::Marker m;
    m.header.frame_id = world_frame_; m.header.stamp = now();
    m.ns = "runtime_trajectory"; m.id = 0; m.type = m.LINE_STRIP; m.action = m.ADD;
    m.pose.orientation.w = 1.0; m.scale.x = 0.18;
    m.color.r = 0.0f; m.color.g = 1.0f; m.color.b = 0.25f; m.color.a = 1.0f;
    for (const auto& point : simulation_.runtime().output().planning.trajectory.points)
      m.points.push_back(make_point(point.pose.x, point.pose.y, 0.12));
    ma.markers.push_back(std::move(m));
    plan_pub_->publish(ma);
  }

  // ========== LIDAR 点云 ==========
  void publish_lidar() {
    double cx = simulation_.vehicle().x(), cy = simulation_.vehicle().y(), cyaw = simulation_.vehicle().yaw();
    auto obstacles = map_->to_obstacles();
    // 合并随机障碍
    for (const auto& ob : obstacle_manager_.obstacles())
      obstacles.push_back(Obstacle{Vec2{ob.x, ob.y}, ob.radius});

    sensor_msgs::msg::PointCloud2 cloud;
    cloud.header.frame_id = world_frame_; cloud.header.stamp = now();
    cloud.height = 1; cloud.width = LIDAR_BEAMS;
    sensor_msgs::PointCloud2Modifier mod(cloud);
    mod.setPointCloud2FieldsByString(1, "xyz");
    mod.resize(LIDAR_BEAMS);
    sensor_msgs::PointCloud2Iterator<float> it_x(cloud, "x");
    sensor_msgs::PointCloud2Iterator<float> it_y(cloud, "y");
    sensor_msgs::PointCloud2Iterator<float> it_z(cloud, "z");

    for (int i = 0; i < LIDAR_BEAMS; ++i, ++it_x, ++it_y, ++it_z) {
      double theta = cyaw + 2.0 * M_PI * i / LIDAR_BEAMS;
      double dx = std::cos(theta), dy = std::sin(theta);
      double dist = LIDAR_RANGE; double hit_t = -1.0;
      for (const auto& ob : obstacles) {
        double ox = ob.position.x - cx, oy = ob.position.y - cy;
        double b = dx * ox + dy * oy;
        double c = ox * ox + oy * oy - ob.radius * ob.radius;
        double disc = b * b - c;
        if (disc >= 0.0) {
          double t = -b - std::sqrt(disc);
          if (t > 0.2 && t < LIDAR_RANGE && (hit_t < 0.0 || t < hit_t)) hit_t = t;
        }
      }
      if (hit_t > 0.0) dist = hit_t;
      double rr = dist + std::sin(sim_time_ * 37.0 + i * 1.3) * 0.02;
      if (rr < 0.2) rr = 0.2;
      *it_x = cx + dx * rr; *it_y = cy + dy * rr; *it_z = 0.3;
    }
    lidar_pub_->publish(cloud);
  }

  // ========== 小车可视化 ==========
  void publish_car_marker() {
    visualization_msgs::msg::MarkerArray ma;
    double cx = simulation_.vehicle().x(), cy = simulation_.vehicle().y(), cyaw = simulation_.vehicle().yaw();

    // 车体
    {
      visualization_msgs::msg::Marker m;
      m.header.frame_id = world_frame_; m.header.stamp = now();
      m.ns = "car"; m.id = 0;
      m.type = m.CUBE; m.action = m.ADD;
      m.pose.position.x = cx; m.pose.position.y = cy; m.pose.position.z = scenario_height_at(cx, cy) + 0.5;
      m.pose.orientation.z = std::sin(cyaw / 2.0); m.pose.orientation.w = std::cos(cyaw / 2.0);
      m.scale.x = 1.8; m.scale.y = 0.9; m.scale.z = 0.6;
      m.color.r = 0.1f; m.color.g = 0.4f; m.color.b = 0.9f; m.color.a = 1.0f;
      ma.markers.push_back(m);
    }
    // 速度矢量
    {
      visualization_msgs::msg::Marker m;
      m.header.frame_id = world_frame_; m.header.stamp = now();
      m.ns = "car"; m.id = 1;
      m.type = m.ARROW; m.action = m.ADD;
      m.pose.position.x = cx; m.pose.position.y = cy; m.pose.position.z = scenario_height_at(cx, cy) + 1.2;
      m.pose.orientation.z = std::sin(cyaw / 2.0); m.pose.orientation.w = std::cos(cyaw / 2.0);
      m.scale.x = 0.6 + std::fabs(simulation_.vehicle().speed()) * 0.3; m.scale.y = 0.25; m.scale.z = 0.25;
      switch (last_action_) {
        case Action::kAccelerate: m.color.r = 0; m.color.g = 1; m.color.b = 0; break;
        case Action::kCruise:     m.color.r = 0; m.color.g = 0.6f; m.color.b = 1; break;
        case Action::kBrake:      m.color.r = 1; m.color.g = 0.6f; m.color.b = 0; break;
        case Action::kStop:       m.color.r = 1; m.color.g = 0; m.color.b = 0; break;
      }
      m.color.a = 1.0f;
      ma.markers.push_back(m);
    }
    // 行驶轨迹
    if (trail_.size() > 2) {
      visualization_msgs::msg::Marker m;
      m.header.frame_id = world_frame_; m.header.stamp = now();
      m.ns = "planning"; m.id = 1;
      m.type = m.LINE_STRIP; m.action = m.ADD;
      m.pose.orientation.w = 1.0; m.scale.x = 0.08;
      m.color.r = 0; m.color.g = 0.8f; m.color.b = 1; m.color.a = 0.6f;
      m.points.assign(trail_.begin(), trail_.end());
      ma.markers.push_back(m);
    }
    live_pub_->publish(ma);
  }

  // ========== 静态道路重发 ==========
  void add_mission_zone_markers() {
    if (scenario_definition_.id != "mining_haul" ||
        scenario_definition_.reference_route.size() < 6) return;
    const auto add_zone = [this](const char* ns, int id, std::size_t route_index,
                                 const char* label, float r, float g, float b) {
      const auto& p = scenario_definition_.reference_route[route_index];
      visualization_msgs::msg::Marker zone;
      zone.header.frame_id = world_frame_;
      zone.ns = ns; zone.id = id;
      zone.type = visualization_msgs::msg::Marker::CYLINDER;
      zone.action = visualization_msgs::msg::Marker::ADD;
      zone.pose.position.x = p.x; zone.pose.position.y = p.y;
      zone.pose.position.z = scenario_height_at(p.x, p.y) + 0.05;
      zone.pose.orientation.w = 1.0;
      zone.scale.x = 4.0; zone.scale.y = 4.0; zone.scale.z = 0.08;
      zone.color.r = r; zone.color.g = g; zone.color.b = b; zone.color.a = 0.42f;
      extra_markers_.markers.push_back(zone);

      visualization_msgs::msg::Marker text;
      text.header.frame_id = world_frame_;
      text.ns = ns; text.id = id + 100;
      text.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
      text.action = visualization_msgs::msg::Marker::ADD;
      text.pose.position.x = p.x; text.pose.position.y = p.y;
      text.pose.position.z = scenario_height_at(p.x, p.y) + 1.8;
      text.scale.z = 0.8;
      text.color.r = r; text.color.g = g; text.color.b = b; text.color.a = 1.0f;
      text.text = label;
      extra_markers_.markers.push_back(text);
    };
    add_zone("mission_load", 10, scenario_definition_.default_mission.load_index,
             "LOAD", 0.1f, 0.9f, 0.35f);
    add_zone("mission_dump", 20, scenario_definition_.default_mission.dump_index,
             "DUMP", 1.0f, 0.55f, 0.1f);
    add_zone("mission_parking", 30, scenario_definition_.default_mission.parking_index,
             "PARK", 0.25f, 0.65f, 1.0f);
  }

  void publish_road_markers() {
    for (auto& mk : road_markers_.markers) {
      mk.header.frame_id = world_frame_;
      mk.header.stamp = now();
    }
    road_pub_->publish(road_markers_);
    // 额外标记（路况标签等）
    for (auto& mk : extra_markers_.markers) {
      mk.header.frame_id = world_frame_;
      mk.header.stamp = now();
    }
    if (!extra_markers_.markers.empty()) road_pub_->publish(extra_markers_);
  }

  // ========== TF ==========
  void publish_car_tf() {
    double cx = simulation_.vehicle().x(), cy = simulation_.vehicle().y(), cyaw = simulation_.vehicle().yaw();
    geometry_msgs::msg::TransformStamped tf;
    tf.header.stamp = now();
    tf.header.frame_id = world_frame_;
    tf.child_frame_id = base_frame_;
    tf.transform.translation.x = cx; tf.transform.translation.y = cy;
    tf.transform.translation.z = scenario_height_at(cx, cy);
    tf.transform.rotation.z = std::sin(cyaw / 2.0);
    tf.transform.rotation.w = std::cos(cyaw / 2.0);
    tf_broadcaster_->sendTransform(tf);
  }

  // ========== 成员 ==========
  std::unique_ptr<ScenarioMap> map_;
  scenario::ScenarioDefinition scenario_definition_;
  uint64_t scenario_mission_sequence_{0};
  simulation::SimulationEngine simulation_;
  ObstacleManager obstacle_manager_;

  Mode mode_{Mode::kAuto};
  double manual_throttle_{0.0};
  double manual_steer_{0.0};

  double sim_time_{0.0};
  double next_status_log_s_{1.0};
  double sim_dt_{0.05};
  double configured_loop_hz_{20.0};
  double last_loop_duration_ms_{0.0};
  uint64_t message_sequence_{0};
  std::string robot_id_{"car01"};
  std::string world_frame_{"world"};
  std::string base_frame_{"car_base_link"};
  double road_resend_accum_{0.0};
  double last_front_dist_{LIDAR_RANGE};
  Action last_action_{Action::kCruise};
  bool last_planning_success_{true};
  int last_safety_action_{0};
  std::string last_planning_reason_{"not_run"};
  bool   paused_{false};
  bool   driving_{true};
  bool   fixed_ring_obstacles_{true};
  bool   auto_recover_safety_{true};
  double dynamic_obstacle_update_accum_{0.0};

  std::deque<geometry_msgs::msg::Point> trail_;
  visualization_msgs::msg::MarkerArray road_markers_;
  visualization_msgs::msg::MarkerArray extra_markers_;

  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr road_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr live_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr obstacle_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr plan_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr speed_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr action_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr distance_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr obstacle_pub2_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr mode_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_pub_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr runtime_state_pub_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr fault_count_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr mission_stage_pub_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr mission_state_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr mission_progress_pub_;
  rclcpp::Publisher<self_driving_car_demo::msg::Trajectory>::SharedPtr trajectory_pub_;
  rclcpp::Publisher<self_driving_car_demo::msg::ControlCommand>::SharedPtr control_command_pub_;
  rclcpp::Publisher<self_driving_car_demo::msg::RuntimeStatus>::SharedPtr status_pub_;
  rclcpp::Publisher<self_driving_car_demo::msg::FaultArray>::SharedPtr faults_pub_;
  rclcpp::Publisher<self_driving_car_demo::msg::RuntimeMetrics>::SharedPtr metrics_pub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr health_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr recovery_service_;

  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr start_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr pause_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr clear_sub_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr set_mode_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr manual_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr reset_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr estop_sub_;
  rclcpp_action::Server<ExecuteMission>::SharedPtr mission_action_server_;
  std::shared_ptr<MissionGoalHandle> active_mission_goal_;

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr lidar_timer_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<RingRoadSimNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
