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
"""Speed-dependent safety override (Section 3.3), ROS wrapper."""

import rclpy
from amr_core import load_fleet
from amr_core.safety import SafetyMonitor
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool


class SafetyOverrideNode(Node):
    def __init__(self):
        super().__init__("safety_override_node")

        self.declare_parameter("robot_name", "")
        self.declare_parameter("release_hysteresis", 0.15)
        self.declare_parameter("watchdog_rate", 10.0)

        robot_name = self.get_parameter("robot_name").value
        if not robot_name:
            raise RuntimeError(
                "safety_override_node requires a 'robot_name' parameter naming "
                "an entry in amr_core/config/fleet.yaml")

        self._robot = load_fleet().robot(robot_name)
        self._monitor = SafetyMonitor(
            self._robot,
            release_hysteresis=float(self.get_parameter("release_hysteresis").value))

        self._cmd = Twist()
        self._speed = 0.0
        self._min_range = float("inf")
        self._last_scan_time = None

        scan_qos = QoSProfile(depth=5)
        scan_qos.reliability = ReliabilityPolicy.BEST_EFFORT

        self.create_subscription(Twist, "cmd_vel_smoothed_local", self._on_cmd, 10)
        self.create_subscription(Odometry, "odometry/filtered", self._on_odom, 10)
        self.create_subscription(LaserScan, "scan_fixed", self._on_scan, scan_qos)
        self.create_subscription(Bool, "yield_stop", self._on_yield, 10)

        self._pub = self.create_publisher(Twist, "cmd_vel", 10)
        self._status_pub = self.create_publisher(Bool, "safety_stop_active", 10)

        self._yield = False
        self.create_timer(
            1.0 / float(self.get_parameter("watchdog_rate").value), self._tick)

        self.get_logger().info(
            f"safety override up for {self._robot.name}: "
            f"d_safe = {self._monitor.k}*v^2 + {self._monitor.d_min}, "
            f"forward cone +-{self._robot.safety_sector_half_angle_deg} deg")

    def _on_cmd(self, msg: Twist):
        self._cmd = msg

    def _on_odom(self, msg: Odometry):
        self._speed = abs(msg.twist.twist.linear.x)

    def _on_yield(self, msg: Bool):
        self._yield = bool(msg.data)

    def _on_scan(self, msg: LaserScan):
        self._min_range = self._monitor.min_range_in_sector(
            msg.ranges, msg.angle_min, msg.angle_increment,
            msg.range_min, msg.range_max)
        self._last_scan_time = self.get_clock().now()

    def _tick(self):
        if self._last_scan_time is None:
            age = float("inf")
        else:
            age = (self.get_clock().now() - self._last_scan_time).nanoseconds / 1e9

        decision = self._monitor.evaluate(
            speed=self._speed, min_range=self._min_range,
            scan_age=age, yield_stop=self._yield)

        out = self._cmd
        if decision.halt:
            # Translation is refused, rotation is not. Turning in place is the
            # only way out of a halt: the state machine needs min_range to
            # reach d_safe + hysteresis before it releases, and a robot that
            # may not turn cannot change what is in its forward sector, so a
            # close obstacle used to wedge it permanently - it sat at 0.30 m
            # emitting "holding" until the goal timed out.
            #
            # Safe because the sweep radius is smaller than anything the scan
            # can report. Returns below inscribed_radius + 0.05 are discarded
            # as self-returns, so the nearest obstacle the monitor can see at
            # the edge of the forward sector is 0.51 m from base centre on
            # amr1 (0.40 m on amr2) against a circumscribed radius of 0.43 m
            # (0.32 m) - the body never sweeps into it.
            out = Twist()
            out.angular.z = self._cmd.angular.z
        self._pub.publish(out)
        self._status_pub.publish(Bool(data=decision.halt))

        if decision.halt:
            self.get_logger().warn(
                f"SAFETY HALT ({decision.reason}): min_range="
                f"{decision.min_range:.2f}m d_safe={decision.d_safe:.2f}m "
                f"v={self._speed:.2f}m/s", throttle_duration_sec=1.0)


def main():
    rclpy.init()
    node = SafetyOverrideNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
