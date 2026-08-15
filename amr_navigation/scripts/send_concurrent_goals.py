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
"""Dispatch concurrent navigation goals to both robots."""

import math
import os
import time

import rclpy
import yaml
from action_msgs.msg import GoalStatus
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node


_FEEDBACK_LOG_PERIOD_SEC = 3.0


def _yaw_to_quaternion(yaw: float):
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


_MAX_GOAL_RETRIES = 10
_GOAL_RETRY_DELAY_SEC = 5.0


class RobotGoalDispatcher:
    """Owns one robot's NavigateToPose action client and reports progress."""

    def __init__(self, node: Node, robot_name: str, location_name: str,
                 x: float, y: float, yaw: float):
        self._node = node
        self._robot_name = robot_name
        self._location_name = location_name
        self._last_feedback_log = 0.0
        self.done = False
        self._retries = 0
        self.succeeded = False

        self._client = ActionClient(node, NavigateToPose, f"/{robot_name}/navigate_to_pose")

        self._goal_pose = PoseStamped()
        self._goal_pose.header.frame_id = "map"
        self._goal_pose.pose.position.x = x
        self._goal_pose.pose.position.y = y
        qx, qy, qz, qw = _yaw_to_quaternion(yaw)
        self._goal_pose.pose.orientation.x = qx
        self._goal_pose.pose.orientation.y = qy
        self._goal_pose.pose.orientation.z = qz
        self._goal_pose.pose.orientation.w = qw

    def dispatch(self):
        """Send the goal without blocking."""
        self._node.get_logger().info(
            f"[{self._robot_name}] waiting for navigate_to_pose action server..."
        )
        self._client.wait_for_server()
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = self._goal_pose
        self._node.get_logger().info(
            f"[{self._robot_name}] -> '{self._location_name}' "
            f"({self._goal_pose.pose.position.x}, {self._goal_pose.pose.position.y})"
        )
        send_future = self._client.send_goal_async(goal_msg, feedback_callback=self._on_feedback)
        send_future.add_done_callback(self._on_goal_response)

    def _retry_once(self):
        """One-shot: the timer is cancelled before re-dispatching."""
        for timer in list(getattr(self._node, "timers", [])):
            if timer.callback is self._retry_once:
                timer.cancel()
        self.dispatch()

    def _on_feedback(self, feedback_msg):
        now = time.monotonic()
        if now - self._last_feedback_log < _FEEDBACK_LOG_PERIOD_SEC:
            return
        self._last_feedback_log = now
        remaining = feedback_msg.feedback.distance_remaining
        self._node.get_logger().info(f"[{self._robot_name}] distance remaining: {remaining:.2f} m")

    def _on_goal_response(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self._retries += 1
            if self._retries <= _MAX_GOAL_RETRIES:
                self._node.get_logger().warn(
                    f"[{self._robot_name}] goal rejected (server not active "
                    f"yet); retry {self._retries}/{_MAX_GOAL_RETRIES} in "
                    f"{_GOAL_RETRY_DELAY_SEC:.0f}s")
                self._node.create_timer(
                    _GOAL_RETRY_DELAY_SEC, self._retry_once)
                return
            self._node.get_logger().error(
                f"[{self._robot_name}] goal rejected {self._retries} times, "
                f"giving up")
            self.done = True
            return
        self._node.get_logger().info(f"[{self._robot_name}] goal accepted")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._on_result)

    def _on_result(self, future):
        status = future.result().status
        self.succeeded = status == GoalStatus.STATUS_SUCCEEDED
        outcome = "SUCCEEDED" if self.succeeded else f"FAILED (status={status})"
        self._node.get_logger().info(f"[{self._robot_name}] {outcome}")
        self.done = True


def main(args=None):
    rclpy.init(args=args)
    node = Node("send_concurrent_goals")

    locations_path = os.path.join(
        get_package_share_directory("amr_navigation"), "config", "locations.yaml"
    )
    with open(locations_path) as f:
        locations = yaml.safe_load(f)["locations"]

    heavy_storage = locations["heavy_storage"]
    packing_bay_4 = locations["packing_bay_4"]

    dispatchers = [
        RobotGoalDispatcher(
            node, "amr1", "Heavy Storage",
            heavy_storage["x"], heavy_storage["y"], heavy_storage["yaw"],
        ),
        RobotGoalDispatcher(
            node, "amr2", "Packing Bay 4",
            packing_bay_4["x"], packing_bay_4["y"], packing_bay_4["yaw"],
        ),
    ]

    for dispatcher in dispatchers:
        dispatcher.dispatch()

    while rclpy.ok() and not all(d.done for d in dispatchers):
        rclpy.spin_once(node, timeout_sec=0.5)

    node.get_logger().info("All goals resolved.")
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
