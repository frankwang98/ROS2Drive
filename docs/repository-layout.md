# Repository layout and migration plan

This is one ROS 2 package with two execution boundaries.  Its source layout
should make the boundary visible instead of grouping files only by feature.

```text
include/ and src/
├── domain/                 portable types only
├── runtime/ mission/ safety/ planning/ control/ vehicle/
│                           ROS-free Autonomy Runtime Core
├── scenario/ simulation/ replay/
│                           ROS-free scenarios and deterministic simulation
├── adapters/ros/           Domain <-> ROS conversion and ROS 2 nodes
├── adapters/visualization/ RViz map markers and RViz plugin
├── legacy/                 old demo and ring-specific experimental algorithms
└── apps/                   executable entry points
```

## Current transition state

The first directory migration is complete: `src/ros/` and `src/ros2/` became
`src/adapters/ros/`; teaching simulation and RViz moved to
`src/adapters/simulation/` and `src/adapters/visualization/`; old Demo code is
under `src/legacy/`; executable entry points are in `src/apps/`.  The installed
ROS executable names remain unchanged.  CMake target boundaries are the source
of truth:

| Target | Responsibility | Must not depend on |
|---|---|---|
| `sdc_runtime_core` | Runtime, scenarios, planners, controllers, vehicle model | ROS, RViz, Qt |
| `sdc_ros_adapter` | typed Domain <-> ROS conversion | RViz, simulator map |
| `sdc_core` | teaching map/sensor, behavior-tree adapter, legacy demo support | may depend on runtime core |
| `sdc_hud_panel` | optional RViz UI | Runtime internals |

`agriculture_route_scenario.cpp` and `coverage_path_planner.cpp` now belong to
`sdc_runtime_core`.  This prevents a ROS-free scenario or planner from being
silently coupled to visualization dependencies.

## Safe migration order

1. Keep CMake boundaries correct before and after moving files.
2. Move remaining legacy `motor_controller` and `behavior_tree` support only
   after their Runtime ownership is explicit.
3. Replace the global `sdc_core` name with a boundary-specific target name in
   a dedicated CMake-only change.
4. Preserve installed names such as `ring_road_sim` until launch/API
   compatibility has been migrated.

Each directory-only step must pass `bash scripts/format_cpp.sh --check`, a
package build, and the Runtime unit tests before the next move.
