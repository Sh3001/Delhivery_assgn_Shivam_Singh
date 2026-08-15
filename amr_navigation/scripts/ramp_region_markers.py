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
"""Publishes the ramp footprints as RViz markers (Section 27)."""

import rclpy
from amr_core import load_fleet
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from visualization_msgs.msg import Marker, MarkerArray


class RampRegionMarkers(Node):
    def __init__(self):
        super().__init__("ramp_region_markers")
        qos = QoSProfile(depth=1)
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self._pub = self.create_publisher(MarkerArray, "/ramp_regions", qos)

        fleet = load_fleet()
        ramps = getattr(fleet, "ramps", None) or []
        markers = MarkerArray()
        for i, r in enumerate(ramps):
            m = Marker()
            m.header.frame_id = fleet.global_frame
            m.header.stamp = self.get_clock().now().to_msg()
            m.ns = "ramp_regions"
            m.id = i
            m.type = Marker.CUBE
            m.action = Marker.ADD
            m.pose.position.x = 0.5 * (r["x_min"] + r["x_max"])
            m.pose.position.y = 0.5 * (r["y_min"] + r["y_max"])
            m.pose.position.z = 0.02
            m.pose.orientation.w = 1.0
            m.scale.x = abs(r["x_max"] - r["x_min"])
            m.scale.y = abs(r["y_max"] - r["y_min"])
            m.scale.z = 0.02
            m.color.r, m.color.g, m.color.b, m.color.a = 1.0, 0.5, 0.0, 0.45
            markers.markers.append(m)

        self._pub.publish(markers)
        self.get_logger().info(
            f"published {len(markers.markers)} ramp region marker(s) on /ramp_regions")


def main():
    rclpy.init()
    node = RampRegionMarkers()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
