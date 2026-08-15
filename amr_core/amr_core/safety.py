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
"""Speed-dependent safety stopping decision (Section 3.3)."""

import math


class SafetyState:
    """Explicit safety state machine (Section 5)."""

    SAFE = "SAFE"
    OBSTACLE_APPROACHING = "OBSTACLE_APPROACHING"
    CRITICAL_DISTANCE = "CRITICAL_DISTANCE"
    EMERGENCY_STOP = "EMERGENCY_STOP"
    OBSTACLE_CLEAR = "OBSTACLE_CLEAR"
    RECOVERY = "RECOVERY"

    HALTING = (CRITICAL_DISTANCE, EMERGENCY_STOP, OBSTACLE_CLEAR, RECOVERY)


class SafetyDecision:
    def __init__(self, halt, d_safe, min_range, reason="", state=SafetyState.SAFE):
        self.halt = halt
        self.d_safe = d_safe
        self.min_range = min_range
        self.reason = reason
        self.state = state

    def __repr__(self):
        return (f"SafetyDecision({self.state}, halt={self.halt}, "
                f"d_safe={self.d_safe:.3f}, min_range={self.min_range:.3f}, "
                f"reason={self.reason!r})")


class SafetyMonitor:
    """Decides whether an immediate halt must override the nav command."""

    def __init__(self, model, release_hysteresis=0.15):
        self.k = float(model.safety_k)
        self.d_min = float(model.safety_d_min)
        self.sector_half_angle = math.radians(
            float(model.safety_sector_half_angle_deg))
        self.scan_stale_sec = float(model.safety_scan_stale_sec)
        inscribed = float(getattr(model, "inscribed_radius", 0.0))
        self.self_return_floor = inscribed + 0.05
        if self.self_return_floor >= self.d_min:
            raise ValueError(
                f"self-return floor {self.self_return_floor:.2f} m >= d_min "
                f"{self.d_min:.2f} m: obstacles inside d_min would be filtered "
                f"out as self-returns and the safety override could never fire "
                f"at low speed. Lower inscribed_radius or raise safety_d_min.")

        self.body_half_length = float(getattr(model, "body_half_length", 0.0))
        if self.body_half_length and self.d_min <= self.body_half_length:
            raise ValueError(
                f"safety_d_min {self.d_min:.2f} m does not clear the robot's "
                f"own body (half-length {self.body_half_length:.2f} m): the "
                f"halt would trigger at or after contact. Raise safety_d_min "
                f"above the half-length plus the stopping distance.")
        self.release_hysteresis = float(release_hysteresis)
        self._halted = False
        self.state = SafetyState.SAFE
        self.recovery_clear_cycles = 5
        self._clear_count = 0

    def safe_distance(self, speed):
        return self.k * float(speed) ** 2 + self.d_min

    def min_range_in_sector(self, ranges, angle_min, angle_increment,
                            range_min=0.0, range_max=float("inf")):
        """Smallest valid return within +/- sector_half_angle of straight ahead."""
        best = float("inf")
        for i, r in enumerate(ranges):
            if r is None:
                continue
            r = float(r)
            if math.isnan(r) or math.isinf(r):
                continue
            if r < max(range_min, self.self_return_floor) or r > range_max:
                continue
            angle = angle_min + i * angle_increment
            if abs(angle) <= self.sector_half_angle:
                best = min(best, r)
        return best

    def evaluate(self, speed, min_range, scan_age=0.0, yield_stop=False):
        """Advance the safety state machine one step and return the decision."""
        d_safe = self.safe_distance(speed)

        if scan_age > self.scan_stale_sec:
            self._halted = True
            self._clear_count = 0
            self.state = SafetyState.EMERGENCY_STOP
            return SafetyDecision(True, d_safe, min_range,
                                  f"scan stale ({scan_age:.2f}s)", self.state)

        if yield_stop:
            self._halted = True
            self._clear_count = 0
            self.state = SafetyState.CRITICAL_DISTANCE
            return SafetyDecision(True, d_safe, min_range, "traffic yield",
                                  self.state)

        if self._halted:
            if min_range > d_safe + self.release_hysteresis:
                self._clear_count += 1
                if self._clear_count >= self.recovery_clear_cycles:
                    self._halted = False
                    self._clear_count = 0
                    self.state = SafetyState.SAFE
                    return SafetyDecision(False, d_safe, min_range, "recovered",
                                          self.state)
                self.state = SafetyState.RECOVERY
                return SafetyDecision(
                    True, d_safe, min_range,
                    f"recovering ({self._clear_count}/{self.recovery_clear_cycles})",
                    self.state)
            self._clear_count = 0
            self.state = SafetyState.EMERGENCY_STOP
            return SafetyDecision(True, d_safe, min_range, "holding", self.state)

        if min_range < d_safe:
            self._halted = True
            self._clear_count = 0
            self.state = SafetyState.EMERGENCY_STOP
            return SafetyDecision(
                True, d_safe, min_range,
                f"obstacle {min_range:.2f}m < d_safe {d_safe:.2f}m", self.state)

        if min_range < 2.0 * d_safe:
            self.state = SafetyState.OBSTACLE_APPROACHING
            return SafetyDecision(False, d_safe, min_range,
                                  f"approaching ({min_range:.2f}m)", self.state)

        self.state = SafetyState.SAFE
        return SafetyDecision(False, d_safe, min_range, "clear", self.state)
