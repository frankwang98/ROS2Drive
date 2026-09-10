# ROS Interface Contract

All names are relative. Launch under `robot_namespace:=car01` to obtain `/car01/...`.

## Stable interfaces

| Name | Type | QoS | Purpose |
|---|---|---|---|
| `mission/execute` | `ExecuteMission` action | reliable | submit, preempt, cancel and observe a mission |
| `planning/trajectory` | `Trajectory` | reliable/volatile | dense vehicle-trackable trajectory |
| `control/command` | `ControlCommand` | reliable/volatile | post-safety command |
| `runtime/status` | `RuntimeStatus` | reliable/transient-local | latest runtime and mission snapshot |
| `runtime/faults` | `FaultArray` | reliable/transient-local | latest active faults |
| `runtime/metrics` | `RuntimeMetrics` | reliable/volatile | loop and pipeline metrics |
| `runtime/health` | `std_srvs/Trigger` | service | readiness-style health query |
| `runtime/acknowledge_recovery` | `std_srvs/Trigger` | service | release a latched safety stop after recovery preconditions pass |
| `sdc/emergency_stop` | `std_msgs/Bool` | reliable | temporary explicit software ESTOP input |

The `sdc/*` scalar topics are compatibility outputs for the existing gateway/HUD. New
integrations should use the typed interfaces above. Integer enum ordering is not a stable
external contract; typed status messages expose string names.

## Mission semantics

- `allow_preempt=false`: reject while another mission is active.
- `allow_preempt=true`: terminate the previous action and atomically install the new mission.
- `mission_id` is an idempotency key; the Runtime retains the latest 256 IDs and rejects duplicates.
- Typed status/trajectory/control/fault/metrics from one cycle share a monotonically increasing `sequence`.
- A non-empty Mission `header.frame_id` must equal the configured `world_frame`.
- Route coordinates are expressed in the configured `world_frame`.
- `speed_limit >= 0`, `goal_tolerance > 0`, and a non-empty mission ID are required.
- `NavigateTo` requires at least one pose; `FollowRoute` requires at least two.
- `timeout_s=0` disables mission timeout.

## Namespaces and frames

`robot_id` is identity in payloads. `robot_namespace` isolates ROS names. They are related
but not interchangeable. `world_frame` and `base_frame` are parameters; multi-robot TF
deployments should use unique frame names or a frame-prefix adapter.

## Safety contract

Only `control/command` after SafetyManager arbitration is safe to send to a chassis adapter.
Debug planner/controller output must never be wired directly to actuation. A real vehicle
must independently enforce command timeout, steering/speed limits and hardware ESTOP.

STOP and EMERGENCY_STOP policy actions are latched. `RuntimeStatus.recovery_required`
indicates that arbitration is still holding the stop after the triggering condition has
cleared. `recovery_ready` becomes true only after the configured number of healthy cycles
while stationary; only then can `runtime/acknowledge_recovery` release the software latch.
The service never substitutes for physical ESTOP release or chassis safety authorization.
