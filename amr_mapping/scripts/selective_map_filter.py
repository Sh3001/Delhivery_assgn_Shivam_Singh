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
"""Implements the assignment's "Selective Mapping" challenge for AMR-1:."""

import numpy as np
import rclpy
from geometry_msgs.msg import Point
from nav_msgs.msg import OccupancyGrid, Odometry
from visualization_msgs.msg import Marker, MarkerArray
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

_MAP_QOS = QoSProfile(
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)


class SelectiveMapFilter(Node):
    def __init__(self):
        super().__init__("selective_map_filter")

        self.declare_parameter("world_x_min", -50.0)
        self.declare_parameter("world_x_max", 50.0)
        self.declare_parameter("world_y_min", -40.0)
        self.declare_parameter("world_y_max", 40.0)
        self.declare_parameter("resolution", 0.1)
        self.declare_parameter("high_visit_threshold", 5)
        self.declare_parameter("throttle_period_sec", 60.0)
        self.declare_parameter("visit_footprint_m", 0.5)
        self.declare_parameter("visit_min_travel_m", 0.3)
        self.declare_parameter("occupied_threshold", 65)
        self.declare_parameter("publish_frontier_markers", True)

        self._res = float(self.get_parameter("resolution").value)
        self._master_ox = float(self.get_parameter("world_x_min").value)
        self._master_oy = float(self.get_parameter("world_y_min").value)
        x_max = float(self.get_parameter("world_x_max").value)
        y_max = float(self.get_parameter("world_y_max").value)
        self._master_w = int(round((x_max - self._master_ox) / self._res))
        self._master_h = int(round((y_max - self._master_oy) / self._res))

        self._high_visit_threshold = int(self.get_parameter("high_visit_threshold").value)
        self._throttle_period_sec = float(self.get_parameter("throttle_period_sec").value)
        self._visit_footprint_cells = max(
            1, int(round(float(self.get_parameter("visit_footprint_m").value) / self._res))
        )
        self._visit_min_travel_m = float(self.get_parameter("visit_min_travel_m").value)
        self._occupied_threshold = int(self.get_parameter("occupied_threshold").value)
        self._publish_markers = bool(self.get_parameter("publish_frontier_markers").value)
        self._last_change_count = 0

        self._master_value = np.full((self._master_h, self._master_w), -1, dtype=np.int8)
        self._master_last_publish = np.zeros((self._master_h, self._master_w), dtype=np.float64)
        self._master_visit_count = np.zeros((self._master_h, self._master_w), dtype=np.int32)

        self._last_visit_xy = None
        self._stats_period_sec = 10.0
        self._last_stats_log_sec = 0.0

        self.create_subscription(OccupancyGrid, "map", self._on_map, _MAP_QOS)
        self.create_subscription(Odometry, "odom", self._on_odom, 10)
        self._publisher = self.create_publisher(OccupancyGrid, "map_filtered", _MAP_QOS)
        self._frontier_pub = self.create_publisher(MarkerArray, "frontiers", 1)

        self.get_logger().info(
            f"Selective map filter up: master buffer {self._master_w}x{self._master_h} cells "
            f"@ {self._res} m, high_visit_threshold={self._high_visit_threshold}, "
            f"throttle={self._throttle_period_sec}s"
        )

    def _world_to_master_index(self, x: float, y: float):
        col = int(round((x - self._master_ox) / self._res))
        row = int(round((y - self._master_oy) / self._res))
        return row, col

    def _on_odom(self, msg: Odometry):
        """Mark the region around AMR-1's current position as visited."""
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        if self._last_visit_xy is not None:
            dx = x - self._last_visit_xy[0]
            dy = y - self._last_visit_xy[1]
            if (dx * dx + dy * dy) ** 0.5 < self._visit_min_travel_m:
                return
        self._last_visit_xy = (x, y)

        row, col = self._world_to_master_index(x, y)
        half = self._visit_footprint_cells
        r0, r1 = max(0, row - half), min(self._master_h, row + half + 1)
        c0, c1 = max(0, col - half), min(self._master_w, col + half + 1)
        if r1 > r0 and c1 > c0:
            self._master_visit_count[r0:r1, c0:c1] += 1

    def _log_selection_stats(self, now_sec, is_frontier, low_visit, allow_update, visit_slice):
        """Periodically reports how many cells the policy actually suppressed."""
        if now_sec - self._last_stats_log_sec < self._stats_period_sec:
            return
        self._last_stats_log_sec = now_sec

        total = allow_update.size
        forwarded = int(np.count_nonzero(allow_update))
        suppressed = total - forwarded
        heavily_traversed = int(np.count_nonzero(~low_visit))
        self.get_logger().info(
            f"Selective update: forwarded {forwarded}/{total} cells "
            f"({100.0 * suppressed / total:.1f}% suppressed) | "
            f"frontier cells prioritized: {int(np.count_nonzero(is_frontier))} | "
            f"cells in heavily-traversed regions (visits>={self._high_visit_threshold}): "
            f"{heavily_traversed} | forced by change: {self._last_change_count} | "
            f"max visits so far: {int(visit_slice.max()) if visit_slice.size else 0}"
        )

    def _publish_frontiers(self, is_frontier, origin, res):
        """Publish frontier cells as a MarkerArray for RViz."""
        rows, cols = np.nonzero(is_frontier)
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "frontiers"
        marker.id = 0
        marker.type = Marker.POINTS
        marker.action = Marker.ADD
        marker.scale.x = marker.scale.y = max(res, 0.05)
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = 1.0, 1.0, 0.0, 0.8

        step = max(1, len(rows) // 3000)
        for r, c in zip(rows[::step], cols[::step]):
            marker.points.append(Point(
                x=origin.position.x + (c + 0.5) * res,
                y=origin.position.y + (r + 0.5) * res,
                z=0.05))
        self._frontier_pub.publish(MarkerArray(markers=[marker]))

    def _on_map(self, msg: OccupancyGrid):
        res = msg.info.resolution
        if abs(res - self._res) > 1e-6:
            self.get_logger().warn(
                f"Incoming map resolution {res} != configured {self._res}, skipping"
            )
            return

        w, h = msg.info.width, msg.info.height
        if w == 0 or h == 0:
            return
        data = np.array(msg.data, dtype=np.int8).reshape(h, w)

        row0, col0 = self._world_to_master_index(
            msg.info.origin.position.x, msg.info.origin.position.y
        )
        row1, col1 = row0 + h, col0 + w
        if row0 < 0 or col0 < 0 or row1 > self._master_h or col1 > self._master_w:
            self.get_logger().warn(
                "Incoming map extends past the preallocated master buffer bounds; "
                "clipping. Widen world_x_min/max / world_y_min/max if this recurs."
            )
            row0, col0 = max(0, row0), max(0, col0)
            row1, col1 = min(self._master_h, row1), min(self._master_w, col1)
            if row1 <= row0 or col1 <= col0:
                return
            data = data[: row1 - row0, : col1 - col0]

        value_slice = self._master_value[row0:row1, col0:col1]
        ts_slice = self._master_last_publish[row0:row1, col0:col1]
        visit_slice = self._master_visit_count[row0:row1, col0:col1]

        unknown = data == -1
        known = ~unknown
        adjacent_unknown = np.zeros_like(unknown)
        adjacent_unknown[:-1, :] |= unknown[1:, :]
        adjacent_unknown[1:, :] |= unknown[:-1, :]
        adjacent_unknown[:, :-1] |= unknown[:, 1:]
        adjacent_unknown[:, 1:] |= unknown[:, :-1]
        is_frontier = known & adjacent_unknown

        low_visit = visit_slice < self._high_visit_threshold

        now_sec = self.get_clock().now().nanoseconds / 1e9
        stale_enough = (now_sec - ts_slice) >= self._throttle_period_sec

        was_occupied = value_slice >= self._occupied_threshold
        is_occupied = data >= self._occupied_threshold
        was_known = value_slice >= 0
        significant_change = known & ((was_occupied != is_occupied) | (~was_known))

        allow_update = is_frontier | low_visit | stale_enough | significant_change
        self._last_change_count = int(np.count_nonzero(
            significant_change & ~(is_frontier | low_visit | stale_enough)))

        value_slice[allow_update] = data[allow_update]
        ts_slice[allow_update] = now_sec

        self._log_selection_stats(now_sec, is_frontier, low_visit, allow_update, visit_slice)
        if self._publish_markers:
            self._publish_frontiers(is_frontier, msg.info.origin, res)

        out = OccupancyGrid()
        out.header = msg.header
        out.info = msg.info
        out.info.width = value_slice.shape[1]
        out.info.height = value_slice.shape[0]
        out.data = value_slice.flatten().tolist()
        self._publisher.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = SelectiveMapFilter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
