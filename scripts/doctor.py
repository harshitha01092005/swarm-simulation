#!/usr/bin/env python3
"""Read-only environment diagnosis; no installations or system modifications."""

import importlib.util
import json
import os
import platform
import shutil
import sys


def main():
    commands = {name: shutil.which(name) for name in ("ros2", "gz", "colcon", "docker")}
    ros_ready = bool(commands["ros2"] and commands["gz"] and importlib.util.find_spec("rclpy"))
    report = {
        "system": platform.system(), "architecture": platform.machine(),
        "python": platform.python_version(), "ros_distro": os.getenv("ROS_DISTRO"),
        "commands": commands, "ros_python_available": importlib.util.find_spec("rclpy") is not None,
        "phase_1_runtime_ready": ros_ready,
    }
    print(json.dumps(report, indent=2))
    if not ros_ready:
        print("\nROS/Gazebo runtime is not ready. See INSTALLATION.md.\n"
              "On macOS ROS/Gazebo run in Docker; the desktop runs natively with: bash scripts/start.sh\n"
              "Core tests still run with: python3 -m unittest discover -s tests -v")
    return 0 if ros_ready else 1


if __name__ == "__main__":
    sys.exit(main())
