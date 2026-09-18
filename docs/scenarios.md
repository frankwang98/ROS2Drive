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

Behavior IDs select thresholds only. The independent `scenario` parameter currently accepts
`ring_demo` and `mining_haul`; it changes initial pose, route, default mission, deterministic
obstacles and the simple RViz map. `behavior_profile:=auto` chooses the scenario default,
while an explicit profile allows cross-combination testing.

```bash
ros2 launch self_driving_car_demo ring_road.launch.py scenario:=ring_demo
ros2 launch self_driving_car_demo ring_road.launch.py scenario:=mining_haul
ros2 launch self_driving_car_demo ring_road.launch.py \
  scenario:=mining_haul behavior_profile:=RingDemo
```

The first migration slice is now present: ROS-free `ScenarioDefinition` owns the portable
contract, and `RingScenarioDefinition` supplies the ring initial pose, closed reference
route, default FollowRoute mission, vehicle constraints and default `RingDemo` profile.
RViz markers and ring static-obstacle construction remain in `RingMap` until the adapter
migration is completed.

`mining_haul` is currently a non-closed LOAD-to-DUMP route and carries an elevation profile. This is a visualization/trajectory layer
only: the vehicle dynamics remain planar, so it does not yet model pitch, grade resistance,
or load-dependent uphill acceleration.

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
