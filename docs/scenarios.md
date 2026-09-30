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

## Ring Frenet trajectory planning

The default `scenario:=ring_demo planner_type:=auto` selects `RingLanePlanner`.
Its reference is the origin-centred, counter-clockwise left-lane circle
(`R=24.5 m` in the teaching map). Only this ring planner is upgraded;
mining, port, agriculture and legacy Lattice/EM planners keep their implementations.

Reference progress is `s=R*theta`, periodic modulo `2*pi*R`.
Lateral offset `d=hypot(x,y)-R` is **outward-positive**, preserving the demo's
right-lane convention: `d=0` is the left lane and `d=3 m` the right lane.

The local planner samples reference-line distance and uses
`theta=vehicle_theta+s/R`, `x=(R+d(s))*cos(theta)`,
`y=(R+d(s))*sin(theta)`. A quintic polynomial connects the actual vehicle d
and heading-derived d' to the requested d over the lane-change length L.
Its boundary conditions are `d''(0)=0`, `d(L)=target_d`,
`d'(L)=d''(L)=0`; it replans from the measured pose each cycle.
Yaw and signed curvature come from analytic Cartesian derivatives, including
d' and d'', rather than assuming the circular tangent during lane changes.
Relative time uses accumulated Cartesian path distance and requested speed;
the existing VelocityPlanner still applies curvature and acceleration limits.

### Control the target d

The default target `-1.0` uses the existing obstacle-driven left/right lane
policy. It can interrupt a return to the left if a new blocker approaches.
An explicit target in `[0.0, 3.0]` requests a lateral offset.

```bash
# Automatic lane changes around the fixed ring obstacles
ros2 launch self_driving_car_demo ring_road.launch.py scenario:=ring_demo

# Start by moving to the right lane over 10 reference-line metres
ros2 launch self_driving_car_demo ring_road.launch.py \
  scenario:=ring_demo ring_target_d:=3.0 ring_change_length:=10.0

# Change the target while the node is running (use a double value)
ros2 param set /ring_road_sim ring.target_d 1.5
ros2 param set /ring_road_sim ring.target_d 0.0
ros2 param set /ring_road_sim ring.target_d -1.0
```

With a robot namespace, prefix the node path accordingly.
`ring.change_length` is a startup setting; restart to change it.
The valid range is `0 < L <= 24 m` (the ring's local horizon).
The ring retains its 0.25 m sampling, 24 m horizon and 3 m lane width.
The curvature bound is derived from the configured maximum steering and
wheelbase. Short changes may be infeasible; increase L instead of bypassing checks.

Explicit d requests still obey road-corridor, curvature and inflated-obstacle
checks. The complete sampled polyline is checked, including between samples
and at the start; lane/angle shortcuts no longer skip collisions.
A blocked or infeasible request returns planning failure and the existing
SafetyManager stops the vehicle. For example, forcing `d=0` toward the fixed
left-lane blockers will stop; forcing `d=1.5` can also intersect them.
Automatic recovery remains governed by the existing simulation settings.

This is a circular Frenet local planner, not a generic road-network or
space-time lattice optimizer. Obstacles use the latest position snapshot;
dynamic-motion prediction and matching measured initial lateral acceleration
are not implemented. Exact ROS/RViz end-to-end verification remains necessary.

`sdc_ring_frenet_tests` covers reference s/d geometry, heading/curvature,
angle wrapping, target changes, invalid input, collision/infeasibility, and
closed-loop lane changes using PurePursuit and the full SimulationEngine.

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

