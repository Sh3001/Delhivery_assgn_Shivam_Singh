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
"""BSP sensor validation gateway (Section 4.1) - ROS wrapper."""

import json

import rclpy
from amr_core import RobotInstance, load_fleet
from amr_core.sensor_bsp import Severity, ValidationState, staleness_severity
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu, LaserScan
from std_msgs.msg import String


class SensorBspNode(Node):
    def __init__(self):
        super().__init__("sensor_bsp_node")

        self.declare_parameter("robot_name", "")
        self.declare_parameter("fleet_config", "")
        self.declare_parameter("health_report_period_sec", 5.0)
        self.declare_parameter("imu_out_of_range_policy", "reject")

        robot_name = self.get_parameter("robot_name").value
        if not robot_name:
            raise RuntimeError(
                "sensor_bsp_node requires a 'robot_name' parameter naming an "
                "entry in amr_core/config/fleet.yaml")

        fleet_config = self.get_parameter("fleet_config").value or None
        self._instance = RobotInstance(
            load_fleet(fleet_config).robot(robot_name))
        self._robot = self._instance.config
        self._imu_validator = self._instance.imu_validator
        self._lidar_validator = self._instance.lidar_validator
        self._imu_rate = self._instance.imu_rate
        self._lidar_rate = self._instance.lidar_rate
        self._max_age = float(self._robot.sensor_max_age_sec)
        self._corrected_frame = self._instance.lidar_frame
        policy = str(self.get_parameter("imu_out_of_range_policy").value).lower()
        if policy not in ("reject", "forward"):
            raise ValueError(
                f"imu_out_of_range_policy must be 'reject' or 'forward', "
                f"got {policy!r}")
        self._reject_out_of_range_imu = (policy == "reject")
        self._status = {"imu": ValidationState.VALID, "lidar": ValidationState.VALID}
        self._last_valid = {"imu": None, "lidar": None}

        sensor_qos = QoSProfile(depth=10)
        sensor_qos.reliability = ReliabilityPolicy.BEST_EFFORT

        self.create_subscription(LaserScan, "scan", self._on_scan, sensor_qos)
        self.create_subscription(Imu, "imu", self._on_imu, sensor_qos)
        self._scan_pub = self.create_publisher(LaserScan, "scan_validated", sensor_qos)
        self._imu_pub = self.create_publisher(Imu, "imu_validated", sensor_qos)
        self._health_pub = self.create_publisher(String, "sensor_health", 10)

        self.create_timer(
            float(self.get_parameter("health_report_period_sec").value),
            self._report_health)

        self.get_logger().info(
            f"sensor BSP up for {robot_name} (model {self._robot.model_name}): "
            f"IMU limits |w|<={self._imu_validator.angular_velocity_limits} rad/s "
            f"(out-of-range policy: {policy}), "
            f"proper |a|<={self._imu_validator.max_linear_acceleration} m/s^2; "
            f"LiDAR range [{self._lidar_validator.range_min}, "
            f"{self._lidar_validator.range_max}] m")

    def _age_of(self, stamp):
        msg_time = rclpy.time.Time.from_msg(stamp)
        return (self.get_clock().now() - msg_time).nanoseconds / 1e9

    def _on_imu(self, msg: Imu):
        age = self._age_of(msg.header.stamp)
        self._imu_rate.observe(self._now_sec())
        result = self._imu_validator.validate(
            (msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z),
            (msg.linear_acceleration.x, msg.linear_acceleration.y,
             msg.linear_acceleration.z),
            orientation=(msg.orientation.x, msg.orientation.y,
                         msg.orientation.z, msg.orientation.w),
            frame_id=msg.header.frame_id or None,
        )
        stale = staleness_severity(age, self._max_age)
        severity = max(result.severity, stale)
        state = result.state
        if stale != Severity.OK:
            state = ValidationState.worst([state, ValidationState.STALE])
            self._imu_validator.counts["stale"] += 1

        for bad in result.violations:
            self.get_logger().warn(
                f"[BSP][{self._robot.name}][IMU] angular velocity exceeded "
                f"physical limit: axis={bad.axis} measured={bad.measured:+.3f} rad/s "
                f"limit={bad.limit:.3f} rad/s "
                f"stamp={msg.header.stamp.sec}.{msg.header.stamp.nanosec:09d} "
                f"state={state}",
                throttle_duration_sec=2.0)

        if result.violations and self._reject_out_of_range_imu:
            severity = Severity.REJECT

        self._publish_status("imu", state, severity)

        other = [r for r in result.reasons
                 if not r.startswith(f"{self._robot.name} angular_velocity")]
        if severity == Severity.REJECT:
            detail = ("; ".join(other) if other
                      else ("out-of-range angular velocity" if result.violations
                            else f"stale by {age:.2f}s"))
            self.get_logger().error(
                f"[BSP][{self._robot.name}][IMU] REJECTED (state={state}): {detail}",
                throttle_duration_sec=2.0)
            self._imu_validator.counts["reject"] += 1
            return
        if severity == Severity.WARN and other:
            self.get_logger().warn(
                f"[BSP][{self._robot.name}][IMU] {'; '.join(other)} (state={state})",
                throttle_duration_sec=2.0)

        self._imu_pub.publish(msg)

    def _on_scan(self, msg: LaserScan):
        age = self._age_of(msg.header.stamp)
        self._lidar_rate.observe(self._now_sec())
        raw_frame = msg.header.frame_id
        result = self._lidar_validator.validate(
            msg.ranges, msg.angle_min, msg.angle_max, msg.angle_increment,
            frame_id=raw_frame or None)
        msg.header.frame_id = self._corrected_frame
        stale = staleness_severity(age, self._max_age)
        severity = max(result.severity, stale)
        state = result.state
        if stale != Severity.OK:
            state = ValidationState.worst([state, ValidationState.STALE])
            self._lidar_validator.counts["stale"] += 1

        self._publish_status("lidar", state, severity)

        if severity == Severity.REJECT:
            self.get_logger().error(
                f"[BSP][{self._robot.name}][LiDAR] REJECTED (state={state}): "
                f"{'; '.join(result.reasons) or f'stale by {age:.2f}s'}",
                throttle_duration_sec=2.0)
            return
        if severity == Severity.WARN:
            self.get_logger().warn(
                f"[BSP][{self._robot.name}][LiDAR] "
                f"{'; '.join(result.reasons) or f'ageing by {age:.2f}s'} "
                f"(state={state})",
                throttle_duration_sec=2.0)

        self._scan_pub.publish(msg)

    def _now_sec(self):
        return self.get_clock().now().nanoseconds / 1e9

    def _publish_status(self, sensor, state, severity):
        self._status[sensor] = state
        if severity != Severity.REJECT:
            self._last_valid[sensor] = self._now_sec()

    def _overall_status(self):
        """OK / WARNING / ERROR / STALE for the robot as a whole."""
        states = list(self._status.values())
        if ValidationState.INVALID in states:
            return "ERROR"
        if ValidationState.STALE in states:
            return "STALE"
        if ValidationState.OUT_OF_RANGE in states:
            return "WARNING"
        return "OK"

    def _report_health(self):
        """Machine-readable diagnostics, one JSON line per period."""
        try:
            self._report_health_impl()
        except Exception as exc:                       # noqa: BLE001
            self.get_logger().error(
                f"[BSP][{self._robot.name}] diagnostics failed "
                f"({type(exc).__name__}: {exc}); validation continues",
                throttle_duration_sec=10.0)

    def _report_health_impl(self):
        diag = self._instance.diagnostics()
        imu_hz = self._imu_rate.measured_hz
        lidar_hz = self._lidar_rate.measured_hz
        diag.update({
            "status": self._overall_status(),
            "imu_state": self._status["imu"],
            "lidar_state": self._status["lidar"],
            "imu_hz": round(imu_hz, 2) if imu_hz else None,
            "lidar_hz": round(lidar_hz, 2) if lidar_hz else None,
            "last_valid_imu_sec": self._last_valid["imu"],
            "last_valid_lidar_sec": self._last_valid["lidar"],
            "report_time_sec": self._now_sec(),
        })
        self._health_pub.publish(String(data=json.dumps(diag, sort_keys=True)))

        summary = (
            f"[BSP][{self._robot.name}] status={diag['status']} "
            f"IMU total={diag['imu_total']} rejected={diag['imu_rejected']} "
            f"w-violations={diag['imu_angular_velocity_violations']} "
            f"per-axis={diag['imu_axis_violations']} stale={diag['imu_stale']} | "
            f"LiDAR total={diag['lidar_total']} rejected={diag['lidar_rejected']} "
            f"out-of-range-readings={diag['lidar_out_of_range_readings']} "
            f"stale={diag['lidar_stale']}")

        if diag["status"] in ("ERROR", "STALE"):
            self.get_logger().error(summary)
        elif diag["status"] == "WARNING":
            self.get_logger().warn(summary)
        else:
            self.get_logger().info(summary)


def main():
    rclpy.init()
    node = SensorBspNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
