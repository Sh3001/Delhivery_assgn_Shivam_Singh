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
"""Remove peer ROBOTS from the scan the PLANNER sees."""

import math

import rclpy
from amr_core import load_fleet
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener


class PeerScanFilter(Node):
    def __init__(self):
        super().__init__("peer_scan_filter")

        self.declare_parameter("robot_name", "")
        self.declare_parameter("input_topic", "scan_validated")
        self.declare_parameter("output_topic", "scan_no_peers")
        self.declare_parameter("peer_margin", 0.25)

        self.robot = self.get_parameter("robot_name").value
        if not self.robot:
            raise ValueError("peer_scan_filter requires robot_name")
        self.margin = float(self.get_parameter("peer_margin").value)

        fleet = load_fleet()
        self.peers = {r.name: float(r.footprint_radius) + self.margin
                      for r in fleet.robots if r.name != self.robot}
        if not self.peers:
            self.get_logger().warn(
                "no peers configured - this node is a pass-through")

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        sensor_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=5,
        )
        self.pub = self.create_publisher(
            LaserScan, self.get_parameter("output_topic").value, sensor_qos)
        self.create_subscription(
            LaserScan, self.get_parameter("input_topic").value,
            self.on_scan, sensor_qos)

        self._masked_total = 0
        self._scans = 0
        self.create_timer(10.0, self._report)

        self.get_logger().info(
            f"peer scan filter up for {self.robot}: masking "
            + ", ".join(f"{n} within {r:.2f} m" for n, r in self.peers.items())
            + " (safety scan is left unfiltered)")

    def _peer_xy(self, peer, scan_frame, stamp):
        """Peer centre expressed in the scan's own frame."""
        for when in (stamp, rclpy.time.Time()):
            try:
                tf = self.tf_buffer.lookup_transform(
                    scan_frame, f"{peer}/base_footprint", when)
                return (tf.transform.translation.x, tf.transform.translation.y)
            except Exception:
                continue
        return None

    def on_scan(self, msg: LaserScan):
        stamp = rclpy.time.Time.from_msg(msg.header.stamp)
        peers = []
        for peer, radius in self.peers.items():
            xy = self._peer_xy(peer, msg.header.frame_id, stamp)
            if xy is not None:
                peers.append((xy[0], xy[1], radius))

        out = LaserScan()
        out.header = msg.header
        out.angle_min = msg.angle_min
        out.angle_max = msg.angle_max
        out.angle_increment = msg.angle_increment
        out.time_increment = msg.time_increment
        out.scan_time = msg.scan_time
        out.range_min = msg.range_min
        out.range_max = msg.range_max
        out.intensities = msg.intensities

        if not peers:
            out.ranges = msg.ranges
            self.pub.publish(out)
            return

        ranges = list(msg.ranges)
        masked = 0
        for i, r in enumerate(ranges):
            if not math.isfinite(r) or r <= 0.0:
                continue
            angle = msg.angle_min + i * msg.angle_increment
            ex, ey = r * math.cos(angle), r * math.sin(angle)
            for px, py, radius in peers:
                if math.hypot(ex - px, ey - py) <= radius:
                    ranges[i] = float("inf")
                    masked += 1
                    break

        out.ranges = ranges
        self.pub.publish(out)
        self._masked_total += masked
        self._scans += 1

    def _report(self):
        if self._scans:
            self.get_logger().info(
                f"masked {self._masked_total / self._scans:.1f} peer returns "
                f"per scan over the last {self._scans} scans "
                f"(environment obstacles untouched)")
        self._masked_total = 0
        self._scans = 0


def main():
    rclpy.init()
    node = PeerScanFilter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
