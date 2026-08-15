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
"""Fuses AMR-1's and AMR-2's independent slam_toolbox maps into one shared."""

import math

import numpy as np
from amr_core import load_fleet
import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

_MAP_QOS = QoSProfile(
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)


class RobotMapContribution:
    """Tracks one robot's known spawn pose, used to transform its local."""

    def __init__(self, spawn_x: float, spawn_y: float, spawn_yaw: float):
        self.spawn_x = spawn_x
        self.spawn_y = spawn_y
        self.cos_yaw = math.cos(spawn_yaw)
        self.sin_yaw = math.sin(spawn_yaw)


class MapMergeNode(Node):
    def __init__(self):
        super().__init__("map_merge_node")

        fleet = load_fleet()
        bounds = fleet.map

        self.declare_parameter("resolution", float(bounds.get("resolution", 0.1)))
        self.declare_parameter("global_x_min", float(bounds.get("x_min", -30.0)))
        self.declare_parameter("global_x_max", float(bounds.get("x_max", 28.0)))
        self.declare_parameter("global_y_min", float(bounds.get("y_min", -18.0)))
        self.declare_parameter("global_y_max", float(bounds.get("y_max", 18.0)))
        self.declare_parameter("publish_rate_hz", 1.0)

        for robot in fleet.robots:
            self.declare_parameter(f"{robot.name}_spawn_x", robot.x)
            self.declare_parameter(f"{robot.name}_spawn_y", robot.y)
            self.declare_parameter(f"{robot.name}_spawn_yaw", robot.yaw)

        self._res = float(self.get_parameter("resolution").value)
        self._global_ox = float(self.get_parameter("global_x_min").value)
        self._global_oy = float(self.get_parameter("global_y_min").value)
        x_max = float(self.get_parameter("global_x_max").value)
        y_max = float(self.get_parameter("global_y_max").value)
        self._global_w = int(round((x_max - self._global_ox) / self._res))
        self._global_h = int(round((y_max - self._global_oy) / self._res))

        self._global_grid = np.full((self._global_h, self._global_w), -1, dtype=np.int8)

        self._provenance = np.zeros((self._global_h, self._global_w), dtype=np.uint8)
        self._global_stamp = np.zeros((self._global_h, self._global_w), dtype=np.float64)
        self.declare_parameter("occupied_threshold", 65)
        self.declare_parameter("occupancy_memory_sec", 30.0)
        self._occupied_threshold = int(self.get_parameter("occupied_threshold").value)
        self._occupancy_memory_sec = float(
            self.get_parameter("occupancy_memory_sec").value)
        self._robot_ids = {"amr1": 1, "amr2": 2}
        self._merge_counts = {"amr1": 0, "amr2": 0}
        self._stats_period_sec = 10.0
        self._last_stats_log_sec = 0.0

        self._contributions = {
            "amr1": RobotMapContribution(
                float(self.get_parameter("amr1_spawn_x").value),
                float(self.get_parameter("amr1_spawn_y").value),
                float(self.get_parameter("amr1_spawn_yaw").value),
            ),
            "amr2": RobotMapContribution(
                float(self.get_parameter("amr2_spawn_x").value),
                float(self.get_parameter("amr2_spawn_y").value),
                float(self.get_parameter("amr2_spawn_yaw").value),
            ),
        }

        self.create_subscription(
            OccupancyGrid, "/amr1/map_filtered", lambda m: self._on_robot_map("amr1", m), _MAP_QOS
        )
        self.create_subscription(
            OccupancyGrid, "/amr2/map", lambda m: self._on_robot_map("amr2", m), _MAP_QOS
        )
        self._publisher = self.create_publisher(OccupancyGrid, "/map", _MAP_QOS)
        self._global_publisher = self.create_publisher(
            OccupancyGrid, "/global_map", _MAP_QOS)

        publish_period = 1.0 / float(self.get_parameter("publish_rate_hz").value)
        self.create_timer(publish_period, self._publish_merged_map)

        self.get_logger().info(
            f"Map merge node up: global buffer {self._global_w}x{self._global_h} cells "
            f"@ {self._res} m, spanning x:[{self._global_ox},{x_max}] "
            f"y:[{self._global_oy},{y_max}]"
        )

    def _on_robot_map(self, robot_name: str, msg: OccupancyGrid):
        contribution = self._contributions[robot_name]
        res = msg.info.resolution
        if abs(res - self._res) > 1e-6:
            self.get_logger().warn(
                f"{robot_name} map resolution {res} != configured {self._res}, skipping"
            )
            return

        w, h = msg.info.width, msg.info.height
        if w == 0 or h == 0:
            return
        data = np.array(msg.data, dtype=np.int8).reshape(h, w)
        known_mask = data != -1
        if not known_mask.any():
            return

        local_x = msg.info.origin.position.x + (np.arange(w) + 0.5) * res
        local_y = msg.info.origin.position.y + (np.arange(h) + 0.5) * res
        local_xx, local_yy = np.meshgrid(local_x, local_y)

        world_xx = (
            contribution.spawn_x
            + local_xx * contribution.cos_yaw
            - local_yy * contribution.sin_yaw
        )
        world_yy = (
            contribution.spawn_y
            + local_xx * contribution.sin_yaw
            + local_yy * contribution.cos_yaw
        )

        global_col = np.round((world_xx - self._global_ox) / self._res).astype(np.int64)
        global_row = np.round((world_yy - self._global_oy) / self._res).astype(np.int64)

        in_bounds = (
            (global_row >= 0)
            & (global_row < self._global_h)
            & (global_col >= 0)
            & (global_col < self._global_w)
        )
        write_mask = known_mask & in_bounds

        now = self.get_clock().now().nanoseconds / 1e9
        rows, cols = global_row[write_mask], global_col[write_mask]
        incoming = data[write_mask]
        existing = self._global_grid[rows, cols]
        age = now - self._global_stamp[rows, cols]

        occupied_incoming = incoming >= self._occupied_threshold
        occupied_existing = existing >= self._occupied_threshold
        unknown_existing = existing < 0

        take = (
            unknown_existing                                   # nothing known yet
            | occupied_incoming                                # obstacle always wins
            | (~occupied_existing)                             # existing is free
            | (age > self._occupancy_memory_sec)               # stale obstacle decays
        )

        sel_rows, sel_cols = rows[take], cols[take]
        self._global_grid[sel_rows, sel_cols] = incoming[take]
        self._global_stamp[sel_rows, sel_cols] = now
        self._provenance[rows, cols] = self._robot_ids[robot_name]
        self._merge_counts[robot_name] += 1

    def _publish_merged_map(self):
        out = OccupancyGrid()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = "map"
        out.info.resolution = self._res
        out.info.width = self._global_w
        out.info.height = self._global_h
        out.info.origin.position.x = self._global_ox
        out.info.origin.position.y = self._global_oy
        out.info.origin.orientation.w = 1.0
        out.data = self._global_grid.flatten().tolist()
        self._publisher.publish(out)
        self._global_publisher.publish(out)
        self._log_fusion_stats()

    def _log_fusion_stats(self):
        """Periodically reports how much of the unified map each robot is."""
        now_sec = self.get_clock().now().nanoseconds / 1e9
        if now_sec - self._last_stats_log_sec < self._stats_period_sec:
            return
        self._last_stats_log_sec = now_sec

        amr1_cells = int(np.count_nonzero(self._provenance == 1))
        amr2_cells = int(np.count_nonzero(self._provenance == 2))
        known_cells = amr1_cells + amr2_cells
        total = self._provenance.size
        self.get_logger().info(
            f"Fused map: {known_cells} known cells ({100.0 * known_cells / total:.1f}% of grid) "
            f"| amr1 contributed {amr1_cells} ({self._merge_counts['amr1']} updates), "
            f"amr2 contributed {amr2_cells} ({self._merge_counts['amr2']} updates)"
        )


def main(args=None):
    rclpy.init(args=args)
    node = MapMergeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
