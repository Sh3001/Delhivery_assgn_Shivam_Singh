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
"""Board Support Package style sensor validation (Section 4.1)."""

import math


class Severity:
    """How the gateway ACTS on a message: forward, forward-and-log, withhold."""

    OK = 0
    WARN = 1
    REJECT = 2

    NAMES = {0: "OK", 1: "WARN", 2: "REJECT"}


class ValidationState:
    """WHY a message was judged the way it was."""

    VALID = "VALID"
    OUT_OF_RANGE = "OUT_OF_RANGE"   # physically implausible, message well-formed
    STALE = "STALE"                 # arrived too late to steer on
    INVALID = "INVALID"             # malformed: NaN/inf, bad frame, bad structure

    _RANK = {VALID: 0, OUT_OF_RANGE: 1, STALE: 2, INVALID: 3}

    @classmethod
    def worst(cls, states):
        states = [s for s in states if s]
        if not states:
            return cls.VALID
        return max(states, key=lambda s: cls._RANK.get(s, 0))


class AxisViolation:
    """One axis of one vector outside its configured envelope."""

    __slots__ = ("axis", "measured", "limit", "quantity", "robot")

    def __init__(self, axis, measured, limit, quantity, robot=None):
        self.axis = axis
        self.measured = measured
        self.limit = limit
        self.quantity = quantity
        self.robot = robot

    def __repr__(self):
        return (f"AxisViolation({self.quantity}.{self.axis}="
                f"{self.measured:.3f} > {self.limit:.3f})")

    def describe(self):
        who = f"{self.robot} " if self.robot else ""
        return (f"{who}{self.quantity} axis {self.axis}: measured "
                f"{self.measured:+.3f}, limit {self.limit:.3f}")


class ValidationResult:
    """Outcome of validating one sensor message."""

    def __init__(self, severity=Severity.OK, reasons=None, stats=None,
                 state=None, violations=None):
        self.severity = severity
        self.reasons = reasons or []
        self.stats = stats or {}
        self.state = state or (ValidationState.VALID if severity == Severity.OK
                               else ValidationState.INVALID)
        self.violations = violations or []

    @property
    def ok(self):
        return self.severity == Severity.OK

    @property
    def usable(self):
        """WARN data is still published; REJECT data is withheld."""
        return self.severity != Severity.REJECT

    def __repr__(self):
        return (f"ValidationResult({Severity.NAMES[self.severity]}, "
                f"{self.state}, {self.reasons})")


AXES = ("x", "y", "z")


class FrameCheck:
    """Decides whether a message's frame_id belongs to this robot."""

    def __init__(self, canonical=None, token=None):
        self.canonical = canonical
        self.token = token

    def accepts(self, frame_id):
        if not frame_id:
            return True          # nothing asserted; other checks still apply
        if self.canonical and frame_id == self.canonical:
            return True
        if self.token:
            return self.token in frame_id
        return self.canonical is None

    def describe(self, frame_id):
        return (f"frame_id '{frame_id}' does not belong to this robot "
                f"(expected '{self.canonical}' or a frame containing "
                f"'{self.token}')")


def _axis_limits(raw, fallback_name):
    """Accept either a scalar limit or a per-axis mapping."""
    if raw is None:
        raise ValueError(f"{fallback_name} is not configured")
    if isinstance(raw, dict):
        missing = [a for a in AXES if a not in raw]
        if missing:
            raise ValueError(
                f"{fallback_name} given per-axis but missing {missing}; "
                f"specify all of x, y, z or use a single scalar")
        limits = {a: float(raw[a]) for a in AXES}
    else:
        limits = {a: float(raw) for a in AXES}
    bad = {a: v for a, v in limits.items() if not (v > 0.0) or math.isinf(v)}
    if bad:
        raise ValueError(f"{fallback_name} must be finite and positive, got {bad}")
    return limits


class ImuValidator:
    """Checks IMU samples against the robot model's physical envelope."""

    GRAVITY = 9.81

    def __init__(self, model, expected_frame=None, frame_token=None):
        self.frame_check = FrameCheck(expected_frame, frame_token)
        self.angular_velocity_limits = _axis_limits(
            getattr(model, "imu_max_angular_velocity", None),
            "imu_max_angular_velocity")
        self.max_linear_acceleration = float(model.imu_max_linear_acceleration)
        self.robot_name = getattr(model, "name", "?")
        self.counts = {
            "total": 0, "warn": 0, "reject": 0,
            "angular_velocity_violations": 0, "acceleration_violations": 0,
            "nan": 0, "bad_frame": 0, "stale": 0,
        }
        self.axis_violations = {a: 0 for a in AXES}

    @property
    def max_angular_velocity(self):
        """Back-compatible scalar view: the largest per-axis limit."""
        return max(self.angular_velocity_limits.values())

    def validate(self, angular_velocity, linear_acceleration, orientation=None,
                 frame_id=None):
        """angular_velocity / linear_acceleration are (x, y, z) tuples."""
        self.counts["total"] += 1
        reasons = []
        violations = []
        states = []
        severity = Severity.OK

        for label, vec in (("angular_velocity", angular_velocity),
                           ("linear_acceleration", linear_acceleration)):
            if any(v is None or math.isnan(v) or math.isinf(v) for v in vec):
                reasons.append(f"{label} contains NaN/inf")
                severity = Severity.REJECT
                states.append(ValidationState.INVALID)
                self.counts["nan"] += 1

        if not self.frame_check.accepts(frame_id):
            reasons.append(self.frame_check.describe(frame_id))
            severity = Severity.REJECT
            states.append(ValidationState.INVALID)
            self.counts["bad_frame"] += 1

        if severity != Severity.REJECT:
            for axis, measured in zip(AXES, angular_velocity):
                limit = self.angular_velocity_limits[axis]
                if abs(measured) > limit:
                    violations.append(AxisViolation(
                        axis, measured, limit, "angular_velocity",
                        self.robot_name))
                    self.axis_violations[axis] += 1
            if violations:
                self.counts["angular_velocity_violations"] += 1
                reasons.extend(v.describe() + " rad/s" for v in violations)
                severity = max(severity, Severity.WARN)
                states.append(ValidationState.OUT_OF_RANGE)

            magnitude = math.sqrt(sum(v * v for v in linear_acceleration))
            proper = abs(magnitude - self.GRAVITY)
            if proper > self.max_linear_acceleration:
                reasons.append(
                    f"proper acceleration {proper:.2f} m/s^2 (|a|={magnitude:.2f}, "
                    f"gravity-compensated) exceeds the plausible limit "
                    f"{self.max_linear_acceleration:.2f} m/s^2")
                severity = max(severity, Severity.WARN)
                states.append(ValidationState.OUT_OF_RANGE)
                self.counts["acceleration_violations"] += 1

        if orientation is not None:
            norm = math.sqrt(sum(c * c for c in orientation))
            if norm > 0.0 and abs(norm - 1.0) > 0.05:
                reasons.append(f"orientation quaternion not normalised (|q|={norm:.3f})")
                severity = Severity.REJECT
                states.append(ValidationState.INVALID)

        if severity == Severity.WARN:
            self.counts["warn"] += 1
        elif severity == Severity.REJECT:
            self.counts["reject"] += 1
        return ValidationResult(severity, reasons,
                                state=ValidationState.worst(states),
                                violations=violations)


class LidarValidator:
    """Checks LaserScan samples for structural and physical plausibility."""

    def __init__(self, model, max_invalid_fraction=0.5, expected_frame=None,
                 frame_token=None):
        self.range_min = float(model.lidar_range_min)
        self.range_max = float(model.lidar_range_max)
        self.max_invalid_fraction = float(max_invalid_fraction)
        self.frame_check = FrameCheck(expected_frame, frame_token)
        self.counts = {
            "total": 0, "warn": 0, "reject": 0,
            "out_of_range_readings": 0, "nonfinite_readings": 0,
            "bad_frame": 0, "stale": 0,
        }

    def validate(self, ranges, angle_min=None, angle_max=None,
                 angle_increment=None, frame_id=None):
        self.counts["total"] += 1
        reasons = []
        states = []
        severity = Severity.OK

        n = len(ranges)
        if n == 0:
            self.counts["reject"] += 1
            return ValidationResult(Severity.REJECT, ["scan contains no returns"],
                                    state=ValidationState.INVALID)

        if not self.frame_check.accepts(frame_id):
            self.counts["reject"] += 1
            self.counts["bad_frame"] += 1
            return ValidationResult(
                Severity.REJECT, [self.frame_check.describe(frame_id)],
                state=ValidationState.INVALID)

        if None not in (angle_min, angle_max, angle_increment) and angle_increment > 0:
            expected = int(round((angle_max - angle_min) / angle_increment)) + 1
            if abs(expected - n) > 1:
                reasons.append(
                    f"angle span implies {expected} readings but {n} were sent")
                severity = Severity.REJECT
                states.append(ValidationState.INVALID)

        finite = [r for r in ranges
                  if r is not None and not math.isnan(r) and not math.isinf(r)]
        n_nonfinite = n - len(finite)
        n_out_of_band = sum(1 for r in finite
                            if r < self.range_min or r > self.range_max)
        invalid_fraction = n_out_of_band / float(n)
        self.counts["out_of_range_readings"] += n_out_of_band
        self.counts["nonfinite_readings"] += n_nonfinite

        if invalid_fraction > self.max_invalid_fraction:
            reasons.append(
                f"{100 * invalid_fraction:.0f}% of readings fall outside "
                f"[{self.range_min}, {self.range_max}] m")
            severity = Severity.REJECT
            states.append(ValidationState.OUT_OF_RANGE)
        elif n_out_of_band:
            reasons.append(f"{n_out_of_band}/{n} readings outside the sensor envelope")
            severity = max(severity, Severity.WARN)
            states.append(ValidationState.OUT_OF_RANGE)

        if severity == Severity.WARN:
            self.counts["warn"] += 1
        elif severity == Severity.REJECT:
            self.counts["reject"] += 1
        return ValidationResult(
            severity, reasons,
            {"n": n, "invalid_fraction": invalid_fraction,
             "nonfinite": n_nonfinite, "out_of_range": n_out_of_band},
            state=ValidationState.worst(states))


def staleness_severity(age_sec, max_age_sec):
    """Shared freshness rule."""
    if age_sec > max_age_sec:
        return Severity.REJECT
    if age_sec > 0.5 * max_age_sec:
        return Severity.WARN
    return Severity.OK


class RateMonitor:
    """Tracks publication rate against an expected value."""

    def __init__(self, expected_hz, tolerance=0.5, window=20):
        if expected_hz is not None and expected_hz <= 0:
            raise ValueError(f"expected_hz must be positive, got {expected_hz}")
        self.expected_hz = float(expected_hz) if expected_hz else None
        self.tolerance = float(tolerance)
        self.window = int(window)
        self._stamps = []
        self.violations = 0

    def observe(self, now_sec):
        self._stamps.append(float(now_sec))
        if len(self._stamps) > self.window:
            self._stamps.pop(0)

    @property
    def measured_hz(self):
        if len(self._stamps) < 2:
            return None
        span = self._stamps[-1] - self._stamps[0]
        if span <= 0:
            return None
        return (len(self._stamps) - 1) / span

    def check(self):
        """Return (ok, measured_hz). ok is True when unknown - fail open."""
        if self.expected_hz is None:
            return True, None
        hz = self.measured_hz
        if hz is None:
            return True, None
        floor = self.expected_hz * (1.0 - self.tolerance)
        ok = hz >= floor
        if not ok:
            self.violations += 1
        return ok, hz
