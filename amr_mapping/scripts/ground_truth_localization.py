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
"""Simulation-only localisation aid: anchor the TF chain to Gazebo ground truth."""


import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from tf2_msgs.msg import TFMessage
from tf2_ros import Buffer, TransformListener


def quat_mul(a, b):
    """Hamilton product; quaternions are (x, y, z, w)."""
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    )


def quat_inv(q):
    x, y, z, w = q
    n = x * x + y * y + z * z + w * w
    if n <= 0.0:
        return (0.0, 0.0, 0.0, 1.0)
    return (-x / n, -y / n, -z / n, w / n)


def quat_rot(q, v):
    """Rotate vector v by quaternion q."""
    qv = (v[0], v[1], v[2], 0.0)
    return quat_mul(quat_mul(q, qv), quat_inv(q))[:3]


def compose(t_a, q_a, t_b, q_b):
    """Compose two transforms: apply B first, then A."""
    rotated = quat_rot(q_a, t_b)
    return (tuple(t_a[i] + rotated[i] for i in range(3)), quat_mul(q_a, q_b))


def invert(t, q):
    qi = quat_inv(q)
    ti = quat_rot(qi, t)
    return (tuple(-c for c in ti), qi)


class GroundTruthLocalization(Node):
    def __init__(self):
        super().__init__("ground_truth_localization")

        self.declare_parameter("robots", ["amr1", "amr2"])
        self.declare_parameter("world", "warehouse")
        self.declare_parameter("global_frame", "map")
        self.declare_parameter("publish_rate_hz", 30.0)

        self.robots = list(self.get_parameter("robots").value)
        self.global_frame = self.get_parameter("global_frame").value
        world = self.get_parameter("world").value

        self.truth = {}
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.tf_pub = self.create_publisher(TFMessage, "/tf", 10)

        self.create_subscription(
            TFMessage, f"/world/{world}/dynamic_pose/info", self._on_truth, 10)
        self.create_timer(
            1.0 / float(self.get_parameter("publish_rate_hz").value),
            self._publish)

        self._warned = set()
        self.get_logger().warn(
            f"GROUND-TRUTH LOCALISATION ACTIVE for {self.robots}. This is a "
            f"simulation aid: map->{{robot}}/map is taken from Gazebo, not from "
            f"scan matching. SLAM still runs and still builds the fused map.")

    def _on_truth(self, msg: TFMessage):
        for tr in msg.transforms:
            if tr.child_frame_id in self.robots:
                t = tr.transform.translation
                r = tr.transform.rotation
                self.truth[tr.child_frame_id] = (
                    (t.x, t.y, t.z), (r.x, r.y, r.z, r.w))

    def _publish(self):
        out = []
        now = self.get_clock().now().to_msg()
        for robot in self.robots:
            truth = self.truth.get(robot)
            if truth is None:
                continue
            try:
                tf = self.buffer.lookup_transform(
                    f"{robot}/map", f"{robot}/base_footprint", rclpy.time.Time())
            except Exception:
                if robot not in self._warned:
                    self.get_logger().info(
                        f"waiting for {robot}/map -> {robot}/base_footprint")
                    self._warned.add(robot)
                continue

            t = tf.transform.translation
            r = tf.transform.rotation
            below = ((t.x, t.y, t.z), (r.x, r.y, r.z, r.w))

            inv_t, inv_q = invert(*below)
            anchor_t, anchor_q = compose(truth[0], truth[1], inv_t, inv_q)

            msg = TransformStamped()
            msg.header.stamp = now
            msg.header.frame_id = self.global_frame
            msg.child_frame_id = f"{robot}/map"
            msg.transform.translation.x = anchor_t[0]
            msg.transform.translation.y = anchor_t[1]
            msg.transform.translation.z = anchor_t[2]
            (msg.transform.rotation.x, msg.transform.rotation.y,
             msg.transform.rotation.z, msg.transform.rotation.w) = anchor_q
            out.append(msg)

        if out:
            self.tf_pub.publish(TFMessage(transforms=out))


def main():
    rclpy.init()
    node = GroundTruthLocalization()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
