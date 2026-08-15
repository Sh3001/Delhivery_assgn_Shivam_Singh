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
"""Give the fleet goals, and report whether they were reached."""

import sys
import time

import rclpy
import yaml
from action_msgs.msg import GoalStatus
from ament_index_python.packages import get_package_share_directory
from nav2_msgs.action import NavigateToPose
from nav2_msgs.srv import ClearEntireCostmap
from rclpy.action import ActionClient
from rclpy.node import Node

RETRY_LIMIT = 12
ABORT_RETRY_LIMIT = 4
RETRY_DELAY_SEC = 5.0
PROGRESS_PERIOD_SEC = 5.0


def load_locations():
    """Read the named goals shipped with amr_navigation."""
    path = get_package_share_directory("amr_navigation") + "/config/locations.yaml"
    with open(path) as handle:
        return (yaml.safe_load(handle) or {}).get("locations", {})


def parse_target(text, locations):
    """Turn 'heavy_storage' or '-20.5,-9.5' into (x, y, yaw, label)."""
    if "," in text:
        parts = [p.strip() for p in text.split(",")]
        if len(parts) not in (2, 3):
            raise ValueError(f"coordinates must be 'x,y' or 'x,y,yaw', got {text!r}")
        x, y = float(parts[0]), float(parts[1])
        yaw = float(parts[2]) if len(parts) == 3 else 0.0
        return x, y, yaw, f"({x}, {y})"
    if text not in locations:
        raise ValueError(
            f"unknown location {text!r}; known: {sorted(locations)} "
            f"(or give coordinates as x,y)")
    spot = locations[text]
    return (float(spot["x"]), float(spot["y"]), float(spot.get("yaw", 0.0)),
            f"'{text}' ({spot['x']}, {spot['y']})")


class GoalRunner(Node):
    """One NavigateToPose client per robot, dispatched together."""

    def __init__(self, assignments):
        super().__init__("send_goal")
        self.assignments = assignments
        self.nav_clients = {}
        self.results = {}
        self.retries = {robot: 0 for robot in assignments}
        self.abort_retries = {robot: 0 for robot in assignments}
        for robot in assignments:
            self.nav_clients[robot] = ActionClient(
                self, NavigateToPose, f"/{robot}/navigate_to_pose")

    def clear_costmaps(self, robot):
        """Drop stale marks before planning.

        A robot that spawned beside a peer keeps an obstacle mark from that
        moment: the peer has long since driven away, the scan shows nothing
        within metres and the fused map is free, but the mark sits inside the
        robot's own footprint where no beam can raytrace it away. Its own cell
        stays lethal and the planner refuses to start. Clearing costs nothing -
        the next scan immediately re-marks anything real.
        """
        for scope in ("global_costmap/clear_entirely_global_costmap",
                      "local_costmap/clear_entirely_local_costmap"):
            cli = self.create_client(ClearEntireCostmap, f"/{robot}/{scope}")
            if not cli.wait_for_service(timeout_sec=5.0):
                continue
            future = cli.call_async(ClearEntireCostmap.Request())
            rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)

    def _goal_msg(self, x, y, yaw):
        import math
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = "map"
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        goal.pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal.pose.pose.orientation.w = math.cos(yaw / 2.0)
        return goal

    def dispatch(self, robot):
        x, y, yaw, label = self.assignments[robot]
        if self.retries[robot] == 0:
            self.clear_costmaps(robot)
        client = self.nav_clients[robot]
        if not client.wait_for_server(timeout_sec=15.0):
            self.retries[robot] += 1
            if self.retries[robot] <= RETRY_LIMIT:
                print(f"  [{robot}] waiting for navigate_to_pose "
                      f"({self.retries[robot]}/{RETRY_LIMIT})")
                return self.dispatch(robot)
            print(f"  [{robot}] no navigate_to_pose server after "
                  f"{RETRY_LIMIT} attempts.")
            print(f"  [{robot}] Its Nav2 never activated. Check the fleet "
                  f"terminal for 'Managed nodes are active' - it must appear")
            print(f"  [{robot}] once per robot. If it appears only once, stop "
                  f"and relaunch:")
            print(f"  [{robot}]   ros2 run amr_bringup stop_stack.py")
            self.results[robot] = "NO SERVER"
            return
        future = client.send_goal_async(self._goal_msg(x, y, yaw))
        rclpy.spin_until_future_complete(self, future, timeout_sec=20.0)
        handle = future.result()

        if handle is None or not handle.accepted:
            self.retries[robot] += 1
            if self.retries[robot] <= RETRY_LIMIT:
                print(f"  [{robot}] goal rejected (Nav2 not active yet); "
                      f"retry {self.retries[robot]}/{RETRY_LIMIT} "
                      f"in {RETRY_DELAY_SEC:.0f}s")
                deadline = time.time() + RETRY_DELAY_SEC
                while time.time() < deadline and rclpy.ok():
                    rclpy.spin_once(self, timeout_sec=0.1)
                return self.dispatch(robot)
            print(f"  [{robot}] rejected {self.retries[robot]} times, giving up")
            self.results[robot] = "REJECTED"
            return

        print(f"  [{robot}] accepted -> {label}")
        self.results[robot] = handle.get_result_async()

    def wait(self, timeout_sec):
        pending = {r: f for r, f in self.results.items() if not isinstance(f, str)}
        deadline = time.time() + timeout_sec
        last_report = 0.0
        while pending and time.time() < deadline and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            for robot in list(pending):
                if pending[robot].done():
                    status = pending[robot].result().status
                    name = {
                        GoalStatus.STATUS_SUCCEEDED: "REACHED",
                        GoalStatus.STATUS_ABORTED: "ABORTED",
                        GoalStatus.STATUS_CANCELED: "CANCELLED",
                    }.get(status, f"status={status}")

                    # Nav2 aborts once its recovery budget is spent, but the
                    # costmap has usually changed by then - a moving obstacle
                    # has passed, or a recovery nudged the robot out of
                    # inflated space. Re-planning from the new state normally
                    # succeeds, so an abort is retried rather than reported.
                    if (name != "REACHED"
                            and self.abort_retries[robot] < ABORT_RETRY_LIMIT
                            and time.time() < deadline - 20):
                        self.abort_retries[robot] += 1
                        print(f"  [{robot}] {name} - replanning "
                              f"({self.abort_retries[robot]}/{ABORT_RETRY_LIMIT})")
                        del pending[robot]
                        self.retries[robot] = 0
                        self.clear_costmaps(robot)
                        self.dispatch(robot)
                        if not isinstance(self.results[robot], str):
                            pending[robot] = self.results[robot]
                        continue
                    print(f"  [{robot}] {name}  "
                          f"({time.time() - (deadline - timeout_sec):.0f}s)")
                    self.results[robot] = name
                    del pending[robot]
            now = time.time()
            if now - last_report > PROGRESS_PERIOD_SEC and pending:
                print(f"  ... driving: {', '.join(sorted(pending))}")
                last_report = now
        for robot in pending:
            print(f"  [{robot}] TIMED OUT after {timeout_sec:.0f}s")
            self.results[robot] = "TIMEOUT"


def main():
    args = [a for a in sys.argv[1:] if not a.startswith("--")]
    locations = load_locations()

    if "--list" in sys.argv or not args:
        print("\nNamed goals:\n")
        for name, spot in sorted(locations.items()):
            note = (spot.get("description") or "").strip().split("\n")[0]
            print(f"  {name:<24} ({spot['x']:>6}, {spot['y']:>6})  {note}")
        print("\nUsage:")
        print("  ros2 run amr_bringup send_goal.py amr1=heavy_storage")
        print("  ros2 run amr_bringup send_goal.py amr1=heavy_storage amr2=packing_bay_4")
        print("  ros2 run amr_bringup send_goal.py amr1=-20.5,-9.5\n")
        return 0

    assignments = {}
    for arg in args:
        if "=" not in arg:
            print(f"error: expected robot=goal, got {arg!r}")
            return 2
        robot, target = arg.split("=", 1)
        try:
            assignments[robot] = parse_target(target, locations)
        except ValueError as exc:
            print(f"error: {exc}")
            return 2

    # 600, not 300. amr1 runs a 0.12 m / 0.12 rad StoppedGoalChecker, so it
    # creeps and settles at the end instead of snapping to success the moment
    # it clips the tolerance. On a 20 m route that final settle can outlast a
    # 300 s budget even though the robot is already parked inside tolerance -
    # measured 0.111 m from ramp_side when the old budget expired.
    timeout = 600.0
    for arg in sys.argv[1:]:
        if arg.startswith("--timeout="):
            timeout = float(arg.split("=", 1)[1])

    rclpy.init()
    runner = GoalRunner(assignments)
    print(f"\nSending {len(assignments)} goal(s):")
    for robot in assignments:
        runner.dispatch(robot)
    print()
    runner.wait(timeout)

    print("\nResult:")
    for robot, outcome in sorted(runner.results.items()):
        print(f"  {robot}: {outcome}")
    runner.destroy_node()
    rclpy.shutdown()
    return 0 if all(v == "REACHED" for v in runner.results.values()) else 1


if __name__ == "__main__":
    sys.exit(main())
