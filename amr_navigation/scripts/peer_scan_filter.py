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
from tf2_msgs.msg import TFMessage
from tf2_ros import Buffer, TransformListener


class PeerScanFilter(Node):
    def __init__(self):
        super().__init__("peer_scan_filter")

        self.declare_parameter("robot_name", "")
        self.declare_parameter("input_topic", "scan_validated")
        self.declare_parameter("output_topic", "scan_no_peers")
        self.declare_parameter("peer_margin", 0.25)
        # Names of moving models to keep OUT OF THE MAP as well as the
        # planner costmap. Fleet peers are found from the roster; these are
        # the pedestrians and third-party robots, which SLAM would otherwise
        # bake in as permanent structure - a person who walked past becomes a
        # wall, and the planner refuses a corridor that is actually clear.
        #
        # In simulation their poses are published by Gazebo. On real hardware
        # this is where a dynamic-object detector/tracker belongs; the mask is
        # the same, only the source of the poses differs.
        self.declare_parameter("dynamic_models", [
            "ped_1", "ped_2", "ped_3", "ped_4",
            "robot_1", "robot_2", "robot_3", "robot_4"])
        self.declare_parameter("dynamic_margin", 0.45)
        self.declare_parameter("truth_topic", "/world/warehouse/dynamic_pose/info")

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

        self.dynamic_names = set(self.get_parameter("dynamic_models").value or [])
        self.dynamic_margin = float(self.get_parameter("dynamic_margin").value)
        self.dynamic_world = {}
        # World poses of this robot and its peers, straight from Gazebo. Always
        # subscribed: this is what makes peer masking work before the map TF
        # tree exists, independently of the dynamic-obstacle masking.
        self.robot_world = {}
        self.create_subscription(
            TFMessage, self.get_parameter("truth_topic").value,
            self._on_truth, 10)

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
        # Peers located at least once. Until a peer is in here its TF has
        # never resolved, so masking for it is not actually running yet.
        self._peer_seen = set()
        self._withheld = 0
        self.create_timer(10.0, self._report)

        self.get_logger().info(
            f"peer scan filter up for {self.robot}: masking "
            + ", ".join(f"{n} within {r:.2f} m" for n, r in self.peers.items())
            + " (safety scan is left unfiltered)")

    def _on_truth(self, msg):
        for tr in msg.transforms:
            if tr.child_frame_id in self.dynamic_names:
                self.dynamic_world[tr.child_frame_id] = (
                    tr.transform.translation.x, tr.transform.translation.y)
            # Fleet poses straight from Gazebo. These are the fallback that
            # lets peer masking work before the map TF tree is connected -
            # see _peer_xy_from_truth.
            if tr.child_frame_id == self.robot or tr.child_frame_id in self.peers:
                t = tr.transform.translation
                q = tr.transform.rotation
                yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                                 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
                self.robot_world[tr.child_frame_id] = (t.x, t.y, yaw)

    def _peer_xy_from_truth(self, peer, scan_frame):
        """Peer centre in the scan frame, using Gazebo truth only.

        map->{robot}/map comes from ground_truth_localization, which cannot
        publish until SLAM has published {robot}/map->{robot}/base_footprint,
        which cannot happen until this node feeds SLAM a scan. Resolving the
        peer through TF is therefore impossible at startup. Truth poses break
        that cycle: both robots' world poses arrive on the Gazebo topic
        immediately, and the only transform needed is the robot's own
        base_footprint->scan frame, which robot_state_publisher supplies at
        once because it is internal to the robot.
        """
        me = self.robot_world.get(self.robot)
        them = self.robot_world.get(peer)
        if me is None or them is None:
            return None
        dx, dy = them[0] - me[0], them[1] - me[1]
        cos_y, sin_y = math.cos(me[2]), math.sin(me[2])
        # World delta -> our base_footprint frame (rotate by -yaw).
        bx = dx * cos_y + dy * sin_y
        by = -dx * sin_y + dy * cos_y
        try:
            tf = self.tf_buffer.lookup_transform(
                scan_frame, f"{self.robot}/base_footprint", rclpy.time.Time())
        except Exception:
            return None
        t = tf.transform.translation
        q = tf.transform.rotation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        cos_s, sin_s = math.cos(yaw), math.sin(yaw)
        return (t.x + bx * cos_s - by * sin_s,
                t.y + bx * sin_s + by * cos_s)

    def _dynamic_in_scan_frame(self, scan_frame, stamp):
        """Dynamic-obstacle centres expressed in the scan's frame."""
        out = []
        if not self.dynamic_world:
            return out
        try:
            tf = self.tf_buffer.lookup_transform(
                scan_frame, "map", rclpy.time.Time())
        except Exception:
            return out
        t = tf.transform.translation
        q = tf.transform.rotation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        cos_y, sin_y = math.cos(yaw), math.sin(yaw)
        for (wx, wy) in self.dynamic_world.values():
            x = t.x + wx * cos_y - wy * sin_y
            y = t.y + wx * sin_y + wy * cos_y
            out.append((x, y, self.dynamic_margin))
        return out

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
        unresolved = []
        for peer, radius in self.peers.items():
            xy = self._peer_xy(peer, msg.header.frame_id, stamp)
            if xy is None:
                xy = self._peer_xy_from_truth(peer, msg.header.frame_id)
            if xy is not None:
                self._peer_seen.add(peer)
                peers.append((xy[0], xy[1], radius))
            elif peer not in self._peer_seen:
                unresolved.append(peer)

        # Fail closed while a peer has never been located. This output feeds
        # SLAM, and map->{peer}/base_footprint only exists once
        # ground_truth_localization is up - which happens well after SLAM
        # starts. Publishing an unmasked scan in that window bakes the peer
        # standing in view into the map as permanent structure, and the
        # planner then refuses to start from that cell for the whole run
        # ("Starting point in lethal space"). Dropping the scan only makes
        # SLAM wait for the transform; passing it through corrupts the map.
        if unresolved:
            self._withheld += 1
            self.get_logger().warn(
                "withholding scan: no TF yet for "
                + ", ".join(unresolved) + " - waiting rather than "
                "publishing an unmasked scan into SLAM",
                throttle_duration_sec=5.0)
            return

        peers.extend(self._dynamic_in_scan_frame(msg.header.frame_id, stamp))

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
        if self._withheld:
            self.get_logger().warn(
                f"withheld {self._withheld} scans waiting on peer TF - "
                f"SLAM input is paused, not corrupted")
        self._masked_total = 0
        self._scans = 0
        self._withheld = 0


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
