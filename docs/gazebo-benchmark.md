# Optional Gazebo scene & runtime benchmark

These additions are **opt-in**. Existing `ring_road.launch.py` and `autonomy.launch.py` remain unchanged.

## Gazebo scene (phase 1: integration scaffold)

Requires a ROS 2 distribution with a compatible **Gazebo Sim** and `ros_gz_sim` installation (for example Jazzy + Harmonic). This is **not Gazebo Classic** and is not compatible with the old `gazebo_simcar_ros2` Classic assets without migration.

```bash
source /opt/ros/jazzy/setup.bash
colcon build --packages-select self_driving_car_demo
source install/setup.bash
ros2 launch self_driving_car_demo gazebo.launch.py
# Optionally start the existing native runtime at the same time:
ros2 launch self_driving_car_demo gazebo.launch.py run_native:=true
```

**Important:** the native runtime and Gazebo are currently **independent simulations**. This launch starts a Gazebo world but does **not** send vehicle commands into Gazebo or consume Gazebo odometry. A moving Gazebo vehicle / real sensor feedback loop is **not yet implemented**. This is a safe first step that keeps the existing runtime unchanged.

Next steps for a genuinely coupled backend:
1. Add an original or appropriately licensed Ackermann URDF/SDF model.
2. Introduce a ROS-to-Gazebo actuation adapter with an explicit steering / speed command contract.
3. Feed Gazebo odometry, IMU and LiDAR back into `VehicleInterface` through a separate ROS adapter, rather than running both physics engines.
4. Validate TF frames, units, command timeouts and stop behavior; add Gazebo smoke tests.

## Runtime benchmark (phase 1: topic capture)

Run the existing native simulation in one terminal:

```bash
ros2 launch self_driving_car_demo autonomy.launch.py scenario:=ring_demo use_rviz:=false
```

Then, with the workspace sourced, capture in another terminal:

```bash
python3 benchmark/collect_metrics.py --duration 60 --output benchmark/results/ring.json
# If the runtime is namespaced:
python3 benchmark/collect_metrics.py --namespace car01 --duration 60
```

The collector subscribes to `/runtime/metrics` and `/sdc/odometry` (prefix both with `--namespace` when needed). Confirm actual topic names with `ros2 topic list` before capture. It writes JSON with loop mean / p95 / max, fault sample count, path-point mean and traveled odometry distance. It exits nonzero when no runtime metrics are received.

**This is not yet an algorithm comparison.** It does not report lateral tracking error, obstacle collision rate, or mission success. Such metrics need ground truth and repeatable scenario seeds. Do not claim controller performance based on loop duration alone.

Pure Python summary tests (with a sourced ROS 2 environment):

```bash
python3 -m unittest discover -s benchmark -p 'test_*.py'
```

Record scenario, git SHA, controller, update rate, host CPU and simulation seed alongside results when comparing runs. Avoid committing generated results by default.

## Experimental Ackermann command forwarding

The optional Gazebo scene now contains a minimal four-wheel Ackermann model. Requires compatible `ros_gz_sim` and `ros_gz_bridge`:

```bash
colcon build --packages-select self_driving_car_demo
source install/setup.bash
ros2 launch self_driving_car_demo gazebo.launch.py run_native:=true bridge_commands:=true
```

The ROS command path is `control/command` → Python adapter → `/model/ackermann_car/cmd_vel` → `ros_gz_bridge` → Gazebo AckermannSteering plugin. The adapter maps steering angle and speed to Twist yaw rate with `omega = v*tan(delta)/2.0`, caps inputs, and commands zero on e-stop, braking, invalid values, or stale commands (>0.3 s).

**This is not closed-loop Gazebo simulation.** The native runtime still uses its own physics and odometry. Gazebo wheel contacts, plugin compatibility, and vehicle movement have not been tested in a real ROS/Gazebo environment. Do not interpret this as validated tracking or benchmark data. Next: bridge Gazebo odometry, make the Gazebo state authoritative, and test TF and stop behavior.

## Gazebo odometry observation (experimental)

The Gazebo SDF now includes an OdometryPublisher plugin. The launch bridges its Gazebo Transport topic to ROS 2 `/gazebo/odometry`. Check messages before attempting any closed-loop integration:

```bash
ros2 launch self_driving_car_demo gazebo.launch.py
ros2 topic echo /gazebo/odometry --once
gz topic -l | grep odometry
```

The Gazebo model may need motion commands before publishing useful changing poses. Record independent Gazebo movement with:

```bash
python3 benchmark/collect_metrics.py --duration 60 --odometry-topic /gazebo/odometry --output benchmark/results/gazebo.json
```

The benchmark still requires `/runtime/metrics` from the native runtime. This captures Gazebo displacement but **does not close the feedback loop** or establish ground-truth tracking error. Gazebo odometry must be connected to a dedicated external-state runtime adapter, with internal dynamics disabled, before closed-loop claims are valid. This bridge has not been runtime-tested.

## External physics runtime API (next integration seam)

`SimulationEngine::stepExternal(measured, dt_s)` now accepts a measured `domain::VehicleState` and executes planning/control without applying the resulting command to the native `SimulatedVehicle`. This keeps physics authority with the external simulator. The existing `step(dt_s)` behavior is unchanged.

A core unit test checks that external stepping does not integrate the native vehicle. **The ROS node does not yet select this path**: it still calls `step()`, so this is an API preparation, not a completed Gazebo feedback loop. Next, implement a runtime backend selector, Gazebo odometry subscription, state freshness checks, and ensure all visualization and safety logic use the same measured state.

## Experimental external-state feedback path

With `run_native:=true`, the launch now passes `physics_backend:=gazebo` to the ROS runtime. The node subscribes to `/gazebo/odometry`, checks pose finiteness and freshness (default 0.5 s), and calls `SimulationEngine::stepExternal` instead of advancing the internal dynamics. A separate visualization-only sync keeps legacy RViz markers aligned to the external pose. On stale odometry, the node publishes an emergency-stop command instead of running a planning tick.

```bash
ros2 launch self_driving_car_demo gazebo.launch.py run_native:=true bridge_commands:=true
ros2 topic hz /gazebo/odometry
ros2 topic echo /control/command --once
```

**Validation limitations:** Gazebo SDF steering joints, wheel contact, odometry frame alignment, ROS bridge direction and actual runtime motion are not verified. The native node's manual driving mode and reset service are not supported as Gazebo controls. The SDF model starts at (0,0) while the default ring mission starts at a different position; until the world model is spawned at the scenario initial pose, this is **not a valid ring tracking benchmark**. Use this path for integration smoke tests only, not controller comparisons.
