"""工程化自动驾驶 Runtime 入口。

推荐使用：
  ros2 launch self_driving_car_demo autonomy.launch.py scenario:=mining_haul
旧 ring_road.launch.py 仅作为兼容别名保留。
"""

import os
import runpy


_legacy = runpy.run_path(os.path.join(os.path.dirname(__file__), "ring_road.launch.py"))
generate_launch_description = _legacy["generate_launch_description"]


__all__ = ["generate_launch_description"]
