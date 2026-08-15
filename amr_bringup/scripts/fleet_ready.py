#!/usr/bin/env python3
# Copyright 2026 Delhivery RSE Assignment
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
"""Report when the fleet can actually accept goals, per robot."""

import sys
import time

import rclpy
from amr_core import load_fleet
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node

WAIT_TOTAL_SEC = 180.0
POLL_SEC = 2.0


def main():
    rclpy.init()
    node = Node("fleet_ready")
    robots = [r.name for r in load_fleet().robots]
    clients = {
        name: ActionClient(node, NavigateToPose, f"/{name}/navigate_to_pose")
        for name in robots
    }

    ready = set()
    deadline = time.time() + WAIT_TOTAL_SEC
    while time.time() < deadline and len(ready) < len(robots) and rclpy.ok():
        for name, client in clients.items():
            if name in ready:
                continue
            if client.wait_for_server(timeout_sec=POLL_SEC):
                ready.add(name)
                node.get_logger().info(f"[fleet] {name} ready")
        rclpy.spin_once(node, timeout_sec=0.1)

    bar = "=" * 70
    if len(ready) == len(robots):
        for line in [
            bar,
            f"[fleet] READY - all {len(robots)} robots accept goals.",
            "[fleet] Send one from another terminal:",
            "[fleet]   ros2 run amr_bringup send_goal.py --list",
            "[fleet]   ros2 run amr_bringup send_goal.py "
            "amr1=heavy_storage amr2=packing_bay_4",
            "[fleet]   ros2 run amr_bringup send_goal.py amr1=-24,-6",
            bar,
        ]:
            node.get_logger().info(line)
        code = 0
    else:
        missing = sorted(set(robots) - ready)
        for line in [
            bar,
            f"[fleet] NOT READY - no navigate_to_pose for: {', '.join(missing)}",
            "[fleet] That robot's Nav2 did not activate. This is the known",
            "[fleet] intermittent bringup stall - stop and relaunch:",
            "[fleet]   ros2 run amr_bringup stop_stack.py",
            "[fleet]   ros2 launch amr_bringup fleet.launch.py",
            bar,
        ]:
            node.get_logger().error(line)
        code = 1

    node.destroy_node()
    rclpy.shutdown()
    return code


if __name__ == "__main__":
    sys.exit(main())
