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
"""Drives the warehouse world's dynamic ("third-party robot" / pedestrian."""

import math
import random
import subprocess

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from tf2_msgs.msg import TFMessage

OBSTACLE_ZONES = {
    "ped_1": (-22.0, -17.0, -1.5, 1.5),      # west central corridor (ramp run-up)
    "ped_2": (-21.0, -18.5, 5.0, 11.0),      # west north-south cross aisle
    "ped_3": (0.0, 14.0, 7.0, 9.0),          # north bypass aisle
    "ped_4": (1.0, 7.0, 2.6, 5.0),           # mezzanine deck, north half
    "robot_1": (-25.0, -22.0, -10.5, -9.5),  # west block aisle, clear of the goal
    "robot_2": (0.0, 14.0, -9.0, -7.0),      # south bypass aisle
    "robot_3": (20.0, 24.0, 2.0, 5.0),       # east block, by the packing bay
    "robot_4": (1.0, 7.0, -5.0, -2.6),       # mezzanine deck, south half
}


WORLD_NAME = "warehouse"


def _yaw_from_quaternion(q) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def _wrap_to_pi(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


class WaypointPatrolDriver:
    """Owns one obstacle's cmd_vel publisher and drives it on a randomized."""

    def __init__(
        self,
        node: Node,
        model_name: str,
        zone,
        max_linear_speed: float,
        max_angular_speed: float,
        max_linear_accel: float,
        max_angular_accel: float,
        waypoint_tolerance_m: float,
        control_dt: float,
    ):
        self._node = node
        self._model_name = model_name
        self._zone = zone
        self._max_linear_speed = max_linear_speed
        self._max_angular_speed = max_angular_speed
        self._max_linear_accel = max_linear_accel
        self._max_angular_accel = max_angular_accel
        self._waypoint_tolerance = waypoint_tolerance_m
        self._control_dt = control_dt

        self._have_pose = False
        self._x = 0.0
        self._y = 0.0
        self._z = 0.0
        self._yaw = 0.0
        self._current_linear = 0.0
        self._current_angular = 0.0
        self._target_x, self._target_y = self._pick_target()

        self._stuck_window_ticks = max(1, round(3.0 / control_dt))
        self._stuck_ticks_elapsed = 0
        self._stuck_reference_x = 0.0
        self._stuck_reference_y = 0.0
        self._unstick_cooldown_ticks = 0

        self._publisher = node.create_publisher(Twist, f"/model/{model_name}/cmd_vel", 10)
        node.create_timer(control_dt, self._on_control_tick)

    def _pick_target(self):
        x_min, x_max, y_min, y_max = self._zone
        return random.uniform(x_min, x_max), random.uniform(y_min, y_max)

    def update_pose(self, x: float, y: float, z: float, yaw: float):
        """Call by DynamicObstacleField's single shared TF subscription."""
        self._x = x
        self._y = y
        self._z = z
        self._yaw = yaw
        self._have_pose = True

    def _check_stuck_watchdog(self):
        if self._unstick_cooldown_ticks > 0:
            self._unstick_cooldown_ticks -= 1

        self._stuck_ticks_elapsed += 1
        if self._stuck_ticks_elapsed < self._stuck_window_ticks:
            return
        moved = math.hypot(self._x - self._stuck_reference_x, self._y - self._stuck_reference_y)
        if moved < 0.15 and self._current_linear > 0.05 and self._unstick_cooldown_ticks == 0:
            self._unstick()
            self._unstick_cooldown_ticks = self._stuck_window_ticks
        self._stuck_reference_x = self._x
        self._stuck_reference_y = self._y
        self._stuck_ticks_elapsed = 0

    def _unstick(self):
        self._target_x, self._target_y = self._pick_target()
        req = (
            f'name: "{self._model_name}", '
            f'position: {{x: {self._target_x}, y: {self._target_y}, z: {self._z}}}, '
            f'orientation: {{x: 0, y: 0, z: 0, w: 1}}'
        )
        cmd = [
            "ign", "service", "-s", f"/world/{WORLD_NAME}/set_pose",
            "--reqtype", "ignition.msgs.Pose", "--reptype", "ignition.msgs.Boolean",
            "--timeout", "300", "--req", req,
        ]
        subprocess.Popen(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        self._node.get_logger().warn(
            f"{self._model_name}: no net movement for "
            f"{self._stuck_window_ticks * self._control_dt:.0f}s "
            f"despite active drive - teleporting to a fresh patrol point"
        )

    def _on_control_tick(self):
        if not self._have_pose:
            return  # no pose yet - stay put rather than drive blind

        self._check_stuck_watchdog()

        dx = self._target_x - self._x
        dy = self._target_y - self._y
        distance = math.hypot(dx, dy)

        if distance < self._waypoint_tolerance:
            self._target_x, self._target_y = self._pick_target()
            dx = self._target_x - self._x
            dy = self._target_y - self._y
            distance = math.hypot(dx, dy)

        heading_error = _wrap_to_pi(math.atan2(dy, dx) - self._yaw)

        target_angular = max(
            -self._max_angular_speed,
            min(self._max_angular_speed, 2.0 * heading_error))
        facing_factor = max(0.0, 1.0 - abs(heading_error) / (math.pi / 2.0))
        target_linear = self._max_linear_speed * facing_factor

        self._current_linear = _step_toward(
            self._current_linear, target_linear, self._max_linear_accel * self._control_dt
        )
        self._current_angular = _step_toward(
            self._current_angular, target_angular, self._max_angular_accel * self._control_dt
        )

        twist = Twist()
        twist.linear.x = self._current_linear
        twist.angular.z = self._current_angular
        self._publisher.publish(twist)


def _step_toward(current: float, target: float, max_delta: float) -> float:
    diff = target - current
    if abs(diff) <= max_delta:
        return target
    return current + max_delta * (1.0 if diff > 0.0 else -1.0)


class DynamicObstacleField(Node):
    """Spins up one WaypointPatrolDriver per configured obstacle model."""

    def __init__(self):
        super().__init__("dynamic_obstacle_field")

        self.declare_parameter("obstacle_models", list(OBSTACLE_ZONES.keys()))
        self.declare_parameter("max_linear_speed", 0.6)
        self.declare_parameter("max_angular_speed", 1.0)
        self.declare_parameter("max_linear_accel", 0.4)
        self.declare_parameter("max_angular_accel", 1.5)
        self.declare_parameter("waypoint_tolerance_m", 0.4)
        self.declare_parameter("control_rate_hz", 10.0)

        model_names = self.get_parameter("obstacle_models").value
        max_linear_speed = self.get_parameter("max_linear_speed").value
        max_angular_speed = self.get_parameter("max_angular_speed").value
        max_linear_accel = self.get_parameter("max_linear_accel").value
        max_angular_accel = self.get_parameter("max_angular_accel").value
        waypoint_tolerance_m = self.get_parameter("waypoint_tolerance_m").value
        control_dt = 1.0 / self.get_parameter("control_rate_hz").value

        self._drivers_by_name = {}
        for name in model_names:
            zone = OBSTACLE_ZONES.get(name)
            if zone is None:
                self.get_logger().warn(f"No patrol zone configured for '{name}', skipping")
                continue
            self._drivers_by_name[name] = WaypointPatrolDriver(
                self, name, zone, max_linear_speed, max_angular_speed,
                max_linear_accel, max_angular_accel, waypoint_tolerance_m, control_dt,
            )

        self.create_subscription(
            TFMessage, "/world/warehouse/dynamic_pose/info", self._on_dynamic_pose_info, 10
        )

        self.get_logger().info(
            f"Patrolling {len(self._drivers_by_name)} dynamic obstacle(s): "
            f"{list(self._drivers_by_name)}"
        )

    def _on_dynamic_pose_info(self, msg: TFMessage):
        for transform in msg.transforms:
            driver = self._drivers_by_name.get(transform.child_frame_id)
            if driver is None:
                continue
            t = transform.transform.translation
            driver.update_pose(t.x, t.y, t.z, _yaw_from_quaternion(transform.transform.rotation))


def main(args=None):
    rclpy.init(args=args)
    node = DynamicObstacleField()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
