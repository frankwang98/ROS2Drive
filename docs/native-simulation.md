# Native simulation (macOS / Ubuntu, no ROS required)

Build from the repository root:

```bash
cmake -S native -B build-native -G Ninja -DCMAKE_BUILD_TYPE=Release
cmake --build build-native -j
ctest --test-dir build-native --output-on-failure
./build-native/ros2drive_sim --scenario port_transport --steps 800 --output studio/recording.json
python3 -m http.server 8000
# open http://localhost:8000/studio/
```

The executable uses the same C++ SimulationEngine and VehicleRuntime as the ROS adapter.
The Studio currently reads deterministic simulation recordings; it is not yet a live
ROS/WebSocket bridge. The browser never executes or replaces the C++ planning stack.

Use `--scenario ring_demo|mining_haul|port_transport|agriculture_route`.
Outputs include road lanes, route, obstacles, vehicle pose, speed, steering,
runtime state and planning trajectory. The recording is a local generated file,
not an automatically validated mission success report.

This CMake entry point is intentionally separate from the ROS ament entry point
to preserve existing colcon builds. On Apple Silicon install CMake and Ninja
(e.g. `brew install cmake ninja`). 
