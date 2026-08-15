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
"""Traffic Control Node - conflict detection and yielding protocol (Section 3)."""

import math

import rclpy
from amr_core import load_fleet
from amr_core.conflict import (
    PROCEED, Trajectory, TrajectorySample, TrafficPolicy, YieldState)
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from std_msgs.msg import Bool, Float32, String
from visualization_msgs.msg import Marker, MarkerArray


class TrafficControlNode(Node):
    def __init__(self):
        super().__init__("traffic_control_node")

        self.declare_parameter("check_rate", 5.0)
        self.declare_parameter("plan_stale_sec", 2.0)

        fleet = load_fleet()
        policy = fleet.policy
        self._fleet = fleet
        self._policy = TrafficPolicy(
            margin=float(policy.get("conflict_margin", 0.35)),
            clear_margin=float(policy.get("clear_margin", 1.0)),
            slow_speed_scale=float(policy.get("slow_speed_scale", 0.35)),
        )
        self._horizon = float(policy.get("horizon_seconds", 4.0))
        self._stale = float(self.get_parameter("plan_stale_sec").value)

        self._plans, self._plan_time, self._speeds = {}, {}, {}
        self._scale_pub, self._stop_pub, self._peer_pub = {}, {}, {}

        for robot in fleet.robots:
            name = robot.name
            self.create_subscription(
                Path, f"/{name}/local_plan",
                lambda m, n=name: self._on_plan(n, m), 10)
            self.create_subscription(
                Odometry, f"/{name}/odometry/filtered",
                lambda m, n=name: self._on_odom(n, m), 10)
            self._scale_pub[name] = self.create_publisher(
                Float32, f"/{name}/speed_scale", 10)
            self._stop_pub[name] = self.create_publisher(
                Bool, f"/{name}/yield_stop", 10)
            self._peer_pub[name] = self.create_publisher(
                Path, f"/{name}/peer_trajectory", 10)

        self._state_pub = self.create_publisher(String, "/traffic/state", 10)
        self._marker_pub = self.create_publisher(MarkerArray, "/traffic/conflicts", 1)

        self.create_timer(
            1.0 / float(self.get_parameter("check_rate").value), self._tick)

        order = ", ".join(
            f"{r.name}(prio {r.yield_priority})"
            for r in sorted(fleet.robots, key=lambda r: -r.yield_priority))
        self.get_logger().info(
            f"traffic control up: priority order {order}; lower priority yields. "
            f"horizon {self._horizon}s, margin {self._policy.margin}m")

    def _on_plan(self, name, msg: Path):
        self._plans[name] = msg
        self._plan_time[name] = self.get_clock().now()

    def _on_odom(self, name, msg: Odometry):
        self._speeds[name] = abs(msg.twist.twist.linear.x)

    def _to_trajectory(self, robot, path):
        """Turn a Path into a space-TIME projection."""
        speed = max(self._speeds.get(robot.name, 0.0), 0.05)
        samples, travelled = [], 0.0
        pts = path.poses
        for i, q in enumerate(pts):
            if i:
                travelled += math.hypot(
                    q.pose.position.x - pts[i - 1].pose.position.x,
                    q.pose.position.y - pts[i - 1].pose.position.y)
            t = travelled / speed
            if t > self._horizon:
                break
            samples.append(TrajectorySample(
                q.pose.position.x, q.pose.position.y, t, speed))
        return Trajectory(robot.name, samples,
                          float(robot.footprint_radius), int(robot.yield_priority))

    def _tick(self):
        now = self.get_clock().now()
        trajectories = []
        for robot in self._fleet.robots:
            path = self._plans.get(robot.name)
            stamp = self._plan_time.get(robot.name)
            if path is None or stamp is None:
                continue
            if (now - stamp).nanoseconds / 1e9 > self._stale:
                continue
            traj = self._to_trajectory(robot, path)
            if traj.samples:
                trajectories.append(traj)

        for robot in self._fleet.robots:
            for other in trajectories:
                if other.robot_id != robot.name and other.robot_id in self._plans:
                    self._peer_pub[robot.name].publish(self._plans[other.robot_id])
                    break

        if len(trajectories) < 2:
            for robot in self._fleet.robots:
                self._scale_pub[robot.name].publish(Float32(data=1.0))
                self._stop_pub[robot.name].publish(Bool(data=False))
            return

        directives = self._policy.evaluate(trajectories, speeds=self._speeds)

        for name, (action, scale, reason) in directives.items():
            state = self._policy.states.get(name, YieldState.NORMAL)
            self._scale_pub[name].publish(Float32(data=float(scale)))
            self._stop_pub[name].publish(Bool(data=state == YieldState.WAITING))
            if action != PROCEED:
                self.get_logger().info(f"[{name}] {reason}", throttle_duration_sec=2.0)

        self._state_pub.publish(String(data="; ".join(
            f"{n}={self._policy.states.get(n, YieldState.NORMAL)}"
            for n in self._policy.states)))
        self._publish_conflict_markers(trajectories, directives)

    def _publish_conflict_markers(self, trajectories, directives):
        markers = MarkerArray()
        for i, traj in enumerate(trajectories):
            action = directives.get(traj.robot_id, (PROCEED, 1.0, ""))[0]
            m = Marker()
            m.header.frame_id = self._fleet.global_frame
            m.header.stamp = self.get_clock().now().to_msg()
            m.ns = "projected_trajectory"
            m.id = i
            m.type = Marker.LINE_STRIP
            m.action = Marker.ADD
            m.scale.x = 0.08
            m.color.a = 0.9
            m.color.r = 1.0 if action != PROCEED else 0.1
            m.color.g = 0.1 if action != PROCEED else 1.0
            m.pose.orientation.w = 1.0
            m.points = [Point(x=s.x, y=s.y, z=0.15) for s in traj.samples]
            markers.markers.append(m)
        self._marker_pub.publish(markers)


def main():
    rclpy.init()
    node = TrafficControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
