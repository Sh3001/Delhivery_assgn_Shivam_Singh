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
"""Unit tests for the fleet control logic."""

import math

from amr_core.conflict import (
    PROCEED, SLOW, YIELD, Trajectory, TrajectorySample, TrafficPolicy, find_conflict)
from amr_core.motion_smoothing import MotionSmoother
from amr_core.safety import SafetyMonitor


class FakeModel:
    """Stand-in for a RobotConfig, so tests do not depend on the real YAML."""

    def __init__(self, **kw):
        defaults = dict(
            max_vel_x=1.0, max_vel_theta=2.0,
            max_accel_x=0.5, max_decel_x=0.5, max_accel_theta=1.0,
            max_jerk_x=1.0, max_jerk_theta=2.0,
            speed_jerk_gain=0.0,
            payload_accel_scale=0.5, payload_jerk_scale=0.5,
            safety_k=0.6, safety_d_min=0.35,
            safety_sector_half_angle_deg=40.0, safety_scan_stale_sec=0.5,
            inscribed_radius=0.10, footprint_radius=0.35,
        )
        defaults.update(kw)
        for k, v in defaults.items():
            setattr(self, k, v)


def test_step_demand_becomes_a_ramp():
    """A step input must not appear at the output in one tick."""
    s = MotionSmoother(FakeModel())
    v, _ = s.step(1.0, 0.0, dt=0.05)
    assert v < 0.1, f"first tick jumped to {v}"


def test_acceleration_limit_respected():
    s = MotionSmoother(FakeModel(max_accel_x=0.5, max_jerk_x=1e6))
    dt, prev = 0.05, 0.0
    for _ in range(200):
        v, _ = s.step(1.0, 0.0, dt=dt)
        assert (v - prev) / dt <= 0.5 + 1e-6, "exceeded accel limit"
        prev = v


def test_jerk_limit_respected():
    s = MotionSmoother(FakeModel(max_jerk_x=1.0))
    dt, prev_a = 0.05, 0.0
    for _ in range(200):
        before = s.linear.velocity
        v, _ = s.step(1.0, 0.0, dt=dt)
        a = (v - before) / dt
        assert abs(a - prev_a) <= 1.0 * dt + 1e-6, "exceeded jerk limit"
        prev_a = a


def test_reaches_target_without_overshoot():
    """Converges onto the target without sailing past it."""
    s = MotionSmoother(FakeModel())
    v, peak = 0.0, 0.0
    for _ in range(500):
        v, _ = s.step(0.8, 0.0, dt=0.02)
        peak = max(peak, v)
    assert peak <= 0.8 + 1e-3, f"overshot to {peak}"
    assert abs(v - 0.8) < 1e-3, f"never converged, stuck at {v}"


def test_payload_makes_the_heavy_unit_gentler():
    """Section 3.1: limits must respond to dynamic state (carrying payload)."""
    unloaded, loaded = MotionSmoother(FakeModel()), MotionSmoother(FakeModel())
    for _ in range(10):
        vu, _ = unloaded.step(1.0, 0.0, dt=0.05, loaded=False)
        vl, _ = loaded.step(1.0, 0.0, dt=0.05, loaded=True)
    assert vl < vu, f"loaded ({vl}) should trail unloaded ({vu})"


def test_heavier_model_accelerates_slower_than_lighter():
    """Section 3.1 explicitly requires this ordering between the two types."""
    heavy = MotionSmoother(FakeModel(max_accel_x=0.4, max_jerk_x=0.6))
    light = MotionSmoother(FakeModel(max_accel_x=1.0, max_jerk_x=1.5))
    for _ in range(10):
        vh, _ = heavy.step(1.0, 0.0, dt=0.05)
        vl, _ = light.step(1.0, 0.0, dt=0.05)
    assert vh < vl, f"heavy ({vh}) must trail light ({vl})"


def test_speed_scale_ramps_down_rather_than_cutting():
    """A yield must be a controlled stop, not a discontinuous command."""
    s = MotionSmoother(FakeModel())
    for _ in range(400):
        s.step(0.8, 0.0, dt=0.02)
    cruising = s.linear.velocity
    v, _ = s.step(0.8, 0.0, dt=0.02, speed_scale=0.0)
    assert v > 0.0, "speed_scale=0 cut the command to zero instantly"
    assert v < cruising, "should be decelerating"


def test_d_safe_is_quadratic_in_speed():
    m = SafetyMonitor(FakeModel(safety_k=0.6, safety_d_min=0.35))
    assert abs(m.safe_distance(0.0) - 0.35) < 1e-9
    assert abs(m.safe_distance(1.0) - 0.95) < 1e-9
    assert m.safe_distance(2.0) - 0.35 == 4 * (m.safe_distance(1.0) - 0.35)


def test_halts_inside_threshold_and_holds_until_clear():
    m = SafetyMonitor(FakeModel(), release_hysteresis=0.15)
    assert m.evaluate(speed=0.5, min_range=5.0).halt is False
    assert m.evaluate(speed=0.5, min_range=0.2).halt is True
    d_safe = m.safe_distance(0.5)
    assert m.evaluate(speed=0.5, min_range=d_safe + 0.01).halt is True
    for _ in range(m.recovery_clear_cycles):
        d = m.evaluate(speed=0.5, min_range=d_safe + 0.5)
    assert d.halt is False, "should release after sustained clearance"


def test_stale_scan_fails_safe():
    m = SafetyMonitor(FakeModel(safety_scan_stale_sec=0.5))
    assert m.evaluate(speed=0.5, min_range=99.0, scan_age=2.0).halt is True


def test_forward_sector_only():
    m = SafetyMonitor(FakeModel(safety_sector_half_angle_deg=40.0))
    n = 180
    inc = 2 * math.pi / n
    ranges = [10.0] * n
    ranges[0] = 0.2
    assert m.min_range_in_sector(ranges, -math.pi, inc) == 10.0, \
        "an obstacle behind the robot must not trigger a halt"
    ranges[n // 2] = 0.2
    assert m.min_range_in_sector(ranges, -math.pi, inc) == 0.2


def test_inf_and_nan_are_not_obstacles():
    m = SafetyMonitor(FakeModel())
    r = m.min_range_in_sector([float("inf"), float("nan"), 3.0], -0.1, 0.1)
    assert r == 3.0


def _traj(rid, pts, radius=0.5, priority=50):
    return Trajectory(rid, [TrajectorySample(*p) for p in pts], radius, priority)


def test_paths_crossing_at_different_times_do_not_conflict():
    """The bug the old spatial-only detector had."""
    a = _traj("amr1", [(0, 0, 0.0), (1, 0, 1.0), (2, 0, 2.0)])
    b = _traj("amr2", [(2, -2, 0.0), (2, -1, 1.0), (2, 0, 2.0)])
    assert find_conflict(a, b) is not None
    b_late = _traj("amr2", [(2, -2, 8.0), (2, -1, 9.0), (2, 0, 10.0)])
    assert find_conflict(a, b_late) is None


def test_lower_priority_robot_is_the_one_that_yields():
    """The scout yields; the priority robot is never told to stop."""
    a = _traj("amr1", [(0, 0, 0.0), (1, 0, 1.0)], priority=100)
    b = _traj("amr2", [(1.2, 0, 0.0), (1.0, 0, 1.0)], priority=50)
    p = TrafficPolicy()
    for _ in range(5):
        d = p.evaluate([a, b], speeds={"amr1": 0.5, "amr2": 0.0})
        assert d["amr1"][0] == PROCEED, "the priority robot must not also stop"
    assert d["amr2"][0] == YIELD, d
    assert d["amr2"][1] == 0.0, "a yield is a full stop"


def test_yield_escalates_through_the_state_machine():
    """Section 3: NORMAL -> CONFLICT_DETECTED -> ... -> WAITING."""
    from amr_core.conflict import YieldState
    a = _traj("amr1", [(0, 0, 0.0), (1, 0, 1.0)], priority=100)
    b = _traj("amr2", [(1.2, 0, 0.0), (1.0, 0, 1.0)], priority=50)
    p = TrafficPolicy()
    seen = []
    for _ in range(5):
        p.evaluate([a, b], speeds={"amr1": 0.5, "amr2": 0.0})
        seen.append(p.states["amr2"])
    assert seen[0] == YieldState.CONFLICT_DETECTED, seen
    assert YieldState.YIELD_REQUESTED in seen, seen
    assert YieldState.WAITING in seen, seen
    assert p.states["amr1"] == YieldState.NORMAL


def test_yield_releases_only_after_sustained_clearance():
    """Releasing while the conflict persists would re-collide the pair."""
    from amr_core.conflict import YieldState
    a = _traj("amr1", [(0, 0, 0.0), (1, 0, 1.0)], priority=100)
    b = _traj("amr2", [(1.2, 0, 0.0), (1.0, 0, 1.0)], priority=50)
    clear_a = _traj("amr1", [(0, 30, 0.0), (1, 30, 1.0)], priority=100)
    p = TrafficPolicy()
    for _ in range(5):
        p.evaluate([a, b], speeds={"amr2": 0.0})
    assert p.states["amr2"] == YieldState.WAITING
    p.evaluate([clear_a, b], speeds={"amr2": 0.0})
    assert p.states["amr2"] != YieldState.NORMAL, "resumed on a single clear cycle"
    for _ in range(5):
        p.evaluate([clear_a, b], speeds={"amr2": 0.0})
    assert p.states["amr2"] == YieldState.NORMAL


def test_a_waiting_robot_is_never_held_forever():
    """Deadlock escape: an indefinitely-held robot is released."""
    from amr_core.conflict import YieldState
    a = _traj("amr1", [(0, 0, 0.0), (1, 0, 1.0)], priority=100)
    b = _traj("amr2", [(1.2, 0, 0.0), (1.0, 0, 1.0)], priority=50)
    p = TrafficPolicy()
    p.max_wait_cycles = 5
    states = set()
    for _ in range(30):
        p.evaluate([a, b], speeds={"amr2": 0.0})
        states.add(p.states["amr2"])
    assert YieldState.RESUME in states or YieldState.NORMAL in states, \
        "robot was held in WAITING indefinitely"


def test_distant_conflict_slows_rather_than_stops():
    a = _traj("amr1", [(0, 0, 0.0), (5, 0, 2.5)], priority=100)
    b = _traj("amr2", [(5.2, 0, 0.0), (5.0, 0, 2.5)], priority=50)
    d = TrafficPolicy(hard_yield_seconds=1.0, react_seconds=4.0).evaluate([a, b])
    assert d["amr2"][0] == SLOW, d
    assert 0.0 < d["amr2"][1] < 1.0, "slow should scale speed, not zero it"


def test_no_conflict_means_everyone_proceeds():
    a = _traj("amr1", [(0, 0, 0.0), (1, 0, 1.0)], priority=100)
    b = _traj("amr2", [(0, 20, 0.0), (1, 20, 1.0)], priority=50)
    d = TrafficPolicy().evaluate([a, b])
    assert all(v[0] == PROCEED for v in d.values()), d


def test_returns_from_the_robots_own_body_are_ignored():
    """The LiDAR sits on the robot; returns inside its own radius are chassis."""
    m = SafetyMonitor(FakeModel(inscribed_radius=0.25, safety_d_min=0.45))
    assert m.min_range_in_sector([0.20, 5.0], -0.1, 0.1) == 5.0
    assert m.min_range_in_sector([0.20, 0.6], -0.1, 0.1) == 0.6
    near = m.min_range_in_sector([0.20, 5.0], -0.1, 0.1)
    assert m.evaluate(speed=0.0, min_range=near).halt is False


def test_emergency_stop_requires_sustained_clearance_before_resuming():
    """Section 5: do not resume the instant a reading flickers clear."""
    from amr_core.safety import SafetyState
    m = SafetyMonitor(FakeModel(), release_hysteresis=0.15)
    assert m.evaluate(speed=0.5, min_range=0.1).state == SafetyState.EMERGENCY_STOP

    clear = m.safe_distance(0.5) + 1.0
    for i in range(m.recovery_clear_cycles - 1):
        d = m.evaluate(speed=0.5, min_range=clear)
        assert d.halt is True, f"resumed after only {i + 1} clear cycles"
        assert d.state == SafetyState.RECOVERY
    assert m.evaluate(speed=0.5, min_range=clear).halt is False


def test_a_single_clear_frame_does_not_reset_recovery_progress():
    """A blip back inside d_safe must restart the recovery count."""
    from amr_core.safety import SafetyState
    m = SafetyMonitor(FakeModel())
    m.evaluate(speed=0.5, min_range=0.1)
    clear = m.safe_distance(0.5) + 1.0
    m.evaluate(speed=0.5, min_range=clear)
    m.evaluate(speed=0.5, min_range=0.1)          # obstacle back
    d = m.evaluate(speed=0.5, min_range=clear)    # first clear frame again
    assert d.halt is True and d.state == SafetyState.RECOVERY


def test_approaching_state_warns_before_the_halt():
    """OBSTACLE_APPROACHING is what makes an imminent stop visible."""
    from amr_core.safety import SafetyState
    m = SafetyMonitor(FakeModel())
    d_safe = m.safe_distance(0.5)
    d = m.evaluate(speed=0.5, min_range=1.5 * d_safe)
    assert d.halt is False and d.state == SafetyState.OBSTACLE_APPROACHING
    assert m.evaluate(speed=0.5, min_range=99.0).state == SafetyState.SAFE


def test_self_return_floor_must_not_swallow_d_min():
    """A floor at or above d_min silently disables the override at low speed."""
    import pytest
    with pytest.raises(ValueError, match="self-return floor"):
        SafetyMonitor(FakeModel(inscribed_radius=0.40, safety_d_min=0.35))


def test_obstacle_between_the_floor_and_d_min_still_halts():
    m = SafetyMonitor(FakeModel(inscribed_radius=0.10, safety_d_min=0.35))
    seen = m.min_range_in_sector([0.20, 9.0], -0.1, 0.1)
    assert seen == 0.20, "a real obstacle just outside the body must be visible"
    assert m.evaluate(speed=0.0, min_range=seen).halt is True
