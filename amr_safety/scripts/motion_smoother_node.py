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
"""Jerk- and payload-aware velocity smoothing (Section 3.1), ROS wrapper."""

import rclpy
from amr_core import load_fleet
from amr_core.motion_smoothing import MotionSmoother
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Bool, Float32


class MotionSmootherNode(Node):
    def __init__(self):
        super().__init__("motion_smoother_node")

        self.declare_parameter("robot_name", "")
        self.declare_parameter("control_rate", 30.0)
        self.declare_parameter("cmd_timeout_sec", 0.5)

        robot_name = self.get_parameter("robot_name").value
        if not robot_name:
            raise RuntimeError(
                "motion_smoother_node requires a 'robot_name' parameter naming "
                "an entry in amr_core/config/fleet.yaml")

        self._robot = load_fleet().robot(robot_name)
        self._smoother = MotionSmoother(self._robot)

        self._cmd_timeout = float(self.get_parameter("cmd_timeout_sec").value)
        rate = float(self.get_parameter("control_rate").value)

        self._target = (0.0, 0.0)
        self._last_cmd_time = self.get_clock().now()
        self._loaded = False
        self._speed_scale = 1.0

        self.create_subscription(Twist, "cmd_vel_nav", self._on_cmd, 10)
        self.create_subscription(Bool, "payload_loaded", self._on_payload, 10)
        self.create_subscription(Float32, "speed_scale", self._on_speed_scale, 10)
        self._pub = self.create_publisher(Twist, "cmd_vel_smoothed_local", 10)

        self._dt = 1.0 / rate
        self.create_timer(self._dt, self._tick)

        r = self._robot
        self.get_logger().info(
            f"motion smoother up for {r.name} (model {r.model_name}): "
            f"accel(lin/ang)={r.max_accel_x}/{r.max_accel_theta}, "
            f"jerk(lin/ang)={r.max_jerk_x}/{r.max_jerk_theta}, "
            f"loaded scale accel={r.payload_accel_scale} jerk={r.payload_jerk_scale}")

    def _on_cmd(self, msg: Twist):
        self._target = (msg.linear.x, msg.angular.z)
        self._last_cmd_time = self.get_clock().now()

    def _on_payload(self, msg: Bool):
        self._loaded = bool(msg.data)

    def _on_speed_scale(self, msg: Float32):
        self._speed_scale = max(0.0, min(1.0, float(msg.data)))

    def _tick(self):
        age = (self.get_clock().now() - self._last_cmd_time).nanoseconds / 1e9
        target_v, target_w = (0.0, 0.0) if age > self._cmd_timeout else self._target

        v, w = self._smoother.step(
            target_v, target_w, self._dt,
            loaded=self._loaded, speed_scale=self._speed_scale)

        out = Twist()
        out.linear.x = v
        out.angular.z = w
        self._pub.publish(out)


def main():
    rclpy.init()
    node = MotionSmootherNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
