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
"""Jerk- and acceleration-limited velocity shaping (Section 3.1)."""

import math


class AxisLimiter:
    """One degree of freedom, limited in both acceleration and jerk."""

    def __init__(self, max_accel, max_decel, max_jerk, speed_jerk_gain=0.0):
        self.max_accel = float(max_accel)
        self.max_decel = float(max_decel)
        self.max_jerk = float(max_jerk)
        self.speed_jerk_gain = float(speed_jerk_gain)
        self.velocity = 0.0
        self.accel = 0.0

    def reset(self):
        self.velocity = 0.0
        self.accel = 0.0

    def effective_jerk(self, accel_scale=1.0, jerk_scale=1.0):
        return (self.max_jerk * jerk_scale) / (
            1.0 + self.speed_jerk_gain * abs(self.velocity))

    def step(self, target, dt, accel_scale=1.0, jerk_scale=1.0):
        """Advance one tick toward `target` and return the new velocity."""
        if dt <= 0.0:
            return self.velocity

        speeding_up = abs(target) > abs(self.velocity)
        accel_cap = (self.max_accel if speeding_up else self.max_decel) * accel_scale
        jerk_cap = self.effective_jerk(accel_scale, jerk_scale)

        error = target - self.velocity
        accel_needed = error / dt

        approach_cap = (0.95 * math.sqrt(2.0 * jerk_cap * abs(error))
                        if jerk_cap > 0.0 else accel_cap)

        magnitude = min(abs(accel_needed), accel_cap, approach_cap)
        desired_accel = math.copysign(magnitude, error) if error != 0.0 else 0.0

        max_accel_change = jerk_cap * dt
        delta = desired_accel - self.accel
        delta = max(-max_accel_change, min(max_accel_change, delta))
        self.accel += delta

        new_velocity = self.velocity + self.accel * dt

        self.velocity = new_velocity
        return self.velocity


class MotionSmoother:
    """Linear + angular smoothing for one robot, driven by its model config."""

    def __init__(self, model):
        self.linear = AxisLimiter(
            model.max_accel_x, model.max_decel_x,
            model.max_jerk_x, model.speed_jerk_gain)
        self.angular = AxisLimiter(
            model.max_accel_theta, model.max_accel_theta,
            model.max_jerk_theta, model.speed_jerk_gain)
        self.payload_accel_scale = float(model.payload_accel_scale)
        self.payload_jerk_scale = float(model.payload_jerk_scale)
        self.max_vel_x = float(model.max_vel_x)
        self.max_vel_theta = float(model.max_vel_theta)

    def reset(self):
        self.linear.reset()
        self.angular.reset()

    def step(self, target_v, target_w, dt, loaded=False, speed_scale=1.0):
        """Shape a commanded (v, w) toward the robot's limits."""
        accel_scale = self.payload_accel_scale if loaded else 1.0
        jerk_scale = self.payload_jerk_scale if loaded else 1.0

        scale = max(0.0, min(1.0, float(speed_scale)))
        tv = max(-self.max_vel_x, min(self.max_vel_x, float(target_v))) * scale
        tw = max(-self.max_vel_theta, min(self.max_vel_theta, float(target_w))) * scale

        v = self.linear.step(tv, dt, accel_scale, jerk_scale)
        w = self.angular.step(tw, dt, accel_scale, jerk_scale)
        return v, w
