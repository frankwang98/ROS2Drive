# Gazebo Sim integration (experimental)

Requires a compatible ROS 2 + Gazebo Sim installation with `ros_gz_sim` and `ros_gz_bridge` (for example ROS 2 Jazzy / Gazebo Harmonic). Gazebo Classic is not supported.

## Run

```bash
source /opt/ros/jazzy/setup.bash
colcon build --packages-select self_driving_car_demo
source install/setup.bash
ros2 launch self_driving_car_demo gazebo.launch.py
```

By default, this starts Gazebo, the ROS runtime in `physics_backend:=gazebo` mode, a ROS command adapter and two transport bridges. The Ackermann model is initialized near the `ring_demo` start pose (x=24.5 m, y=0, yaw=pi/2); this only aligns the initial *pose*, not the Gazebo physics fidelity or the scenario geometry. Run Gazebo alone with `run_native:=false bridge_commands:=false`.

Command path: `control/command` → `gazebo_command_bridge.py` → `/model/ackermann_car/cmd_vel` → `ros_gz_bridge` → AckermannSteering. State path: Gazebo OdometryPublisher → `ros_gz_bridge` → `/gazebo/odometry` → ROS runtime `stepExternal()`. The native vehicle integrator is not stepped in external mode. The bridge sends a zero command on emergency stop, braking, invalid input or stale commands (0.3 s). The runtime stops publishing drive commands if odometry is stale (>0.5 s).

## Smoke test

```bash
ros2 topic list | grep -E 'gazebo/odometry|control/command|runtime/metrics'
ros2 topic hz /gazebo/odometry
ros2 topic echo /gazebo/odometry --once
ros2 topic echo /control/command --once
gz topic -l | grep ackermann_car
```

Inspect whether the odometry position changes while commanded to move. Check wheel rotation, steering, frame orientation and ground contact in the Gazebo GUI. Trigger `/sdc/emergency_stop` with `std_msgs/msg/Bool` true and confirm the Gazebo model stops. Kill the odometry bridge and confirm the command watchdog stops the car. **Do not assume these checks have passed; this repository has not been built or run in a Gazebo environment here.**

The legacy RViz visualization reads a synchronized proxy pose. It may not accurately reflect measured speed, steering or altitude, and TF frames need validation. Gazebo reset and manual WASD are not supported. The current ring obstacles are internal to the ROS scenario and are not represented by matching Gazebo collision geometry, so collision statistics are not meaningful yet.

## Benchmark

```bash
python3 benchmark/collect_metrics.py --duration 60 --odometry-topic /gazebo/odometry --output benchmark/results/gazebo.json
python3 benchmark/compare_runs.py benchmark/results/baseline.json benchmark/results/gazebo.json
```

Runtime loop timing and displacement can be compared, but these are not controller tracking accuracy or collision metrics. Avoid comparing different hosts or simulation configurations without documenting the differences.
