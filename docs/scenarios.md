# Scenario Guide

The same Runtime is used in every scenario. A scenario changes Mission data, vehicle limits,
Behavior XML and adapters—not the orchestration code.

| Behavior profile | BehaviorTree ID | Future scenario capability (not implemented yet) |
|---|---|---|
| Ring teaching | `RingDemo` | continuous route, random obstacles |
| Mining | `MiningHaul` | load/unload, haul-road right of way, dump alignment |
| Port | `PortTransport` | quay/yard hand-off, intersection reservation, precise stop |
| Agriculture | `AgricultureRoute` | implement state, crop row, headland turn |

All four scenario trees share the same `scene_driving.xml`; the only difference is the
`stop_t` / `slow_t` / `cruise_t` thresholds injected through the `DistancePolicy` subtree.
Select one explicitly with the `bt_tree_id` ROS/launch parameter. The shared subtree
explicitly remaps `front_dist` and `action`; an unknown ID or malformed XML rejects node
initialization instead of silently selecting another profile.

These IDs currently select behavior thresholds only. The active map, initial pose, route,
obstacle generator and vehicle model are still the ring-road teaching setup. The planned
`scenario` selector will load those environment assets through a ROS-free Scenario Adapter;
it remains separate from `behavior_profile` so profiles can be tested against any scenario.

## Failure scenarios

1. Localization loss: set `FaultInjection.localization_available=false`; expect `FAULT`, zero speed and full brake.
2. Perception dropout: first publish a timestamped obstacle set, stop updates for over 0.5 s; expect watchdog stop.
3. Planning failure: inject a failing `Planner`; expect `PLANNING_FAILED` and no controller bypass.
4. Emergency stop: set the software ESTOP input; expect `ESTOP` and `emergency_stop=true`.
5. Mission timeout: submit a finite timeout and prevent completion; expect `FAILED/mission_timeout`.
6. Preemption: submit a second mission with `allow_preempt=true`; expect the previous action to terminate.

Automated core examples live in `test/runtime_core_test.cpp`. ROS/DDS end-to-end scenarios
remain a user-run verification step until a launch-testing harness is added.

For deterministic replay, enable `SimulationEngine` recording, persist the recorder as
`SDC_REPLAY_V1`, create a Runtime with the same Mission/components, and feed each frame through
`replayFrame()`. Compare Runtime state, safe control and trajectory against `expected_output`.
