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
"""Republishes a LaserScan with its header.frame_id corrected."""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan


class ScanFrameFixer(Node):
    def __init__(self):
        super().__init__("scan_frame_fixer")

        self.declare_parameter("corrected_frame_id", "lidar_link")

        corrected_frame_id = self.get_parameter("corrected_frame_id").value

        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self._publisher = self.create_publisher(LaserScan, "scan_fixed", qos)
        self.create_subscription(LaserScan, "scan", self._on_scan, qos)

        self._corrected_frame_id = corrected_frame_id
        self.get_logger().info(f"Rewriting scan frame_id -> '{corrected_frame_id}'")

    def _on_scan(self, msg: LaserScan):
        msg.header.frame_id = self._corrected_frame_id
        self._publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ScanFrameFixer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
