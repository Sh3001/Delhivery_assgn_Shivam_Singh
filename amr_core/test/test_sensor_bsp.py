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
"""Unit tests for the BSP sensor validation routine (Section 4.1)."""

import pytest
from amr_core.sensor_bsp import (
    ImuValidator, LidarValidator, RateMonitor, Severity,
    ValidationState, staleness_severity)


class FakeModel:
    def __init__(self, **kw):
        defaults = dict(
            name="test_robot",
            imu_max_angular_velocity=2.0,
            imu_max_linear_acceleration=9.0,
            lidar_range_min=0.10,
            lidar_range_max=25.0,
            sensor_max_age_sec=0.5,
        )
        defaults.update(kw)
        for k, v in defaults.items():
            setattr(self, k, v)


def test_normal_imu_sample_passes():
    v = ImuValidator(FakeModel())
    r = v.validate((0.0, 0.0, 0.5), (0.1, 0.0, 9.81))
    assert r.ok, r.reasons


def test_implausible_angular_velocity_warns():
    """The check the assignment calls out explicitly."""
    v = ImuValidator(FakeModel(imu_max_angular_velocity=2.0))
    r = v.validate((0.0, 0.0, 7.5), (0.0, 0.0, 9.81))
    assert r.severity == Severity.WARN
    assert r.state == ValidationState.OUT_OF_RANGE
    assert v.counts["warn"] == 1
    assert v.counts["angular_velocity_violations"] == 1

    assert len(r.violations) == 1
    bad = r.violations[0]
    assert bad.axis == "z"                    # yaw was the offending axis
    assert bad.measured == 7.5
    assert bad.limit == 2.0
    assert bad.quantity == "angular_velocity"
    assert "7.50" in r.reasons[0] and "2.00" in r.reasons[0]
    assert "axis z" in r.reasons[0]


def test_limit_is_per_robot_model_not_fleet_wide():
    """7 rad/s is a fault on the tugger and normal for the scout."""
    heavy = ImuValidator(FakeModel(imu_max_angular_velocity=2.0))
    light = ImuValidator(FakeModel(imu_max_angular_velocity=8.0))
    sample = ((0.0, 0.0, 7.0), (0.0, 0.0, 9.81))
    assert heavy.validate(*sample).severity == Severity.WARN
    assert light.validate(*sample).ok


def test_nan_is_rejected_not_merely_warned():
    v = ImuValidator(FakeModel())
    r = v.validate((0.0, float("nan"), 0.0), (0.0, 0.0, 9.81))
    assert r.severity == Severity.REJECT
    assert not r.usable


def test_unnormalised_quaternion_is_rejected():
    v = ImuValidator(FakeModel())
    r = v.validate((0.0, 0.0, 0.0), (0.0, 0.0, 9.81), orientation=(0.0, 0.0, 0.0, 0.5))
    assert r.severity == Severity.REJECT


def test_excessive_acceleration_warns():
    v = ImuValidator(FakeModel(imu_max_linear_acceleration=9.0))
    r = v.validate((0.0, 0.0, 0.0), (0.0, 0.0, 40.0))
    assert r.severity == Severity.WARN


def test_normal_scan_passes():
    v = LidarValidator(FakeModel())
    assert v.validate([1.0, 2.0, 3.0], -1.0, 1.0, 1.0).ok


def test_inf_returns_are_not_treated_as_faults():
    """inf means 'nothing in range', which is normal in an open aisle."""
    v = LidarValidator(FakeModel())
    r = v.validate([float("inf")] * 10, -1.0, 1.0, 2.0 / 9)
    assert r.ok, r.reasons


def test_mostly_out_of_band_scan_is_rejected():
    v = LidarValidator(FakeModel(lidar_range_max=25.0))
    r = v.validate([900.0] * 10, -1.0, 1.0, 2.0 / 9)
    assert r.severity == Severity.REJECT


def test_a_few_bad_readings_only_warn():
    v = LidarValidator(FakeModel())
    r = v.validate([1.0] * 9 + [900.0], -1.0, 1.0, 2.0 / 9)
    assert r.severity == Severity.WARN
    assert r.usable, "a single bad ray must not discard the whole scan"


def test_angle_span_mismatch_is_rejected():
    """A wrong angle span silently corrupts every bearing downstream."""
    v = LidarValidator(FakeModel())
    r = v.validate([1.0] * 10, angle_min=-1.0, angle_max=1.0, angle_increment=0.01)
    assert r.severity == Severity.REJECT
    assert "angle span" in r.reasons[0]


def test_empty_scan_is_rejected():
    assert LidarValidator(FakeModel()).validate([]).severity == Severity.REJECT


def test_stale_data_is_rejected_and_ageing_data_warns():
    assert staleness_severity(0.1, 0.5) == Severity.OK
    assert staleness_severity(0.4, 0.5) == Severity.WARN
    assert staleness_severity(2.0, 0.5) == Severity.REJECT


def test_counters_track_what_was_seen():
    v = ImuValidator(FakeModel())
    v.validate((0.0, 0.0, 0.1), (0.0, 0.0, 9.8))
    v.validate((0.0, 0.0, 99.0), (0.0, 0.0, 9.8))
    v.validate((float("nan"), 0.0, 0.0), (0.0, 0.0, 9.8))
    assert v.counts["total"] == 3
    assert v.counts["warn"] == 1
    assert v.counts["reject"] == 1
    assert v.counts["angular_velocity_violations"] == 1
    assert v.counts["nan"] == 1
    assert v.axis_violations == {"x": 0, "y": 0, "z": 1}


def test_real_fleet_models_drive_the_validators():
    """The configured robots must actually expose the envelope fields."""
    from amr_core import load_fleet
    for robot in load_fleet().robots:
        imu = ImuValidator(robot)
        lidar = LidarValidator(robot)
        assert imu.max_angular_velocity > 0.0
        assert lidar.range_max > lidar.range_min
        assert imu.validate((0.0, 0.0, 99.0), (0.0, 0.0, 9.81)).severity == Severity.WARN


def test_boundary_value_exactly_at_the_limit_is_accepted():
    """Documented boundary policy: the limit is the edge of the plausible."""
    v = ImuValidator(FakeModel(imu_max_angular_velocity=2.0))
    assert v.validate((0.0, 0.0, 2.0), (0.0, 0.0, 9.81)).ok
    assert v.validate((0.0, 0.0, 2.0000001), (0.0, 0.0, 9.81)).severity == Severity.WARN


def test_negative_value_exceeding_limit_warns():
    """Sign must not launder a violation - the check is on magnitude."""
    v = ImuValidator(FakeModel(imu_max_angular_velocity=2.0))
    r = v.validate((0.0, 0.0, -7.5), (0.0, 0.0, 9.81))
    assert r.severity == Severity.WARN
    assert r.violations[0].measured == -7.5
    assert r.violations[0].axis == "z"


def test_violation_is_attributed_to_the_correct_axis():
    v = ImuValidator(FakeModel(imu_max_angular_velocity=2.0))
    r = v.validate((9.0, 0.0, 0.0), (0.0, 0.0, 9.81))
    assert [b.axis for b in r.violations] == ["x"]
    assert v.axis_violations == {"x": 1, "y": 0, "z": 0}


def test_multiple_axes_each_reported():
    v = ImuValidator(FakeModel(imu_max_angular_velocity=2.0))
    r = v.validate((9.0, -8.0, 0.5), (0.0, 0.0, 9.81))
    assert sorted(b.axis for b in r.violations) == ["x", "y"]


def test_per_axis_limits_are_supported():
    """A tall robot tolerates far less roll rate than yaw rate."""
    v = ImuValidator(FakeModel(
        imu_max_angular_velocity={"x": 0.5, "y": 0.5, "z": 3.0}))
    assert v.validate((0.0, 0.0, 2.5), (0.0, 0.0, 9.81)).ok       # yaw fine
    r = v.validate((1.0, 0.0, 0.0), (0.0, 0.0, 9.81))             # roll not
    assert r.severity == Severity.WARN
    assert r.violations[0].axis == "x" and r.violations[0].limit == 0.5


def test_partial_per_axis_configuration_is_rejected():
    with pytest.raises(ValueError, match="missing"):
        ImuValidator(FakeModel(imu_max_angular_velocity={"x": 1.0}))


def test_nonsense_limits_are_rejected_at_construction():
    for bad in (0.0, -1.0, float("inf")):
        with pytest.raises(ValueError):
            ImuValidator(FakeModel(imu_max_angular_velocity=bad))


def test_infinite_angular_velocity_is_rejected_not_merely_warned():
    v = ImuValidator(FakeModel())
    r = v.validate((float("inf"), 0.0, 0.0), (0.0, 0.0, 9.81))
    assert r.severity == Severity.REJECT
    assert r.state == ValidationState.INVALID


def test_imu_from_another_robot_is_rejected():
    """The multi-robot hazard: AMR-2's data republished as AMR-1's."""
    v = ImuValidator(FakeModel(name="amr1"),
                     expected_frame="amr1/imu_link", frame_token="amr1")
    assert v.validate((0.0, 0.0, 0.1), (0.0, 0.0, 9.81),
                      frame_id="amr1/imu_link").ok
    assert v.validate((0.0, 0.0, 0.1), (0.0, 0.0, 9.81),
                      frame_id="amr1/amr1/base_footprint/amr1/imu_link_sensor").ok
    r = v.validate((0.0, 0.0, 0.1), (0.0, 0.0, 9.81), frame_id="amr2/imu_link")
    assert r.severity == Severity.REJECT
    assert v.counts["bad_frame"] == 1


def test_validation_states_are_distinguished():
    v = ImuValidator(FakeModel(imu_max_angular_velocity=2.0))
    assert v.validate((0.0, 0.0, 0.1), (0.0, 0.0, 9.81)).state == ValidationState.VALID
    assert (v.validate((0.0, 0.0, 9.0), (0.0, 0.0, 9.81)).state
            == ValidationState.OUT_OF_RANGE)
    assert (v.validate((float("nan"), 0.0, 0.0), (0.0, 0.0, 9.81)).state
            == ValidationState.INVALID)
    assert ValidationState.worst(
        [ValidationState.VALID, ValidationState.STALE]) == ValidationState.STALE


def test_lidar_below_minimum_range_is_flagged():
    v = LidarValidator(FakeModel(lidar_range_min=0.10, lidar_range_max=25.0))
    r = v.validate([0.01] * 10, -1.0, 1.0, 2.0 / 9)
    assert r.severity == Severity.REJECT
    assert r.state == ValidationState.OUT_OF_RANGE


def test_lidar_nan_readings_are_counted_but_tolerated():
    """NaN is a legitimate 'no return', not a fault."""
    v = LidarValidator(FakeModel())
    r = v.validate([float("nan")] * 5 + [1.0] * 5, -1.0, 1.0, 2.0 / 9)
    assert r.usable
    assert v.counts["nonfinite_readings"] == 5


def test_lidar_from_another_robot_is_rejected():
    v = LidarValidator(FakeModel(name="amr1"),
                       expected_frame="amr1/lidar_link", frame_token="amr1")
    assert v.validate([1.0] * 10, -1.0, 1.0, 2.0 / 9,
                      frame_id="amr1/lidar_link").ok
    r = v.validate([1.0] * 10, -1.0, 1.0, 2.0 / 9, frame_id="amr2/lidar_link")
    assert r.severity == Severity.REJECT
    assert v.counts["bad_frame"] == 1


def test_lidar_counters_track_readings_not_just_messages():
    v = LidarValidator(FakeModel())
    v.validate([1.0] * 9 + [900.0], -1.0, 1.0, 2.0 / 9)
    v.validate([1.0] * 9 + [900.0], -1.0, 1.0, 2.0 / 9)
    assert v.counts["out_of_range_readings"] == 2
    assert v.counts["total"] == 2


def test_rate_monitor_detects_a_halved_publication_rate():
    """Every message is individually valid; only the interval reveals it."""
    m = RateMonitor(expected_hz=10.0, tolerance=0.2)
    for i in range(10):
        m.observe(i * 0.1)                 # a healthy 10 Hz
    ok, hz = m.check()
    assert ok and abs(hz - 10.0) < 0.1

    slow = RateMonitor(expected_hz=10.0, tolerance=0.2)
    for i in range(10):
        slow.observe(i * 0.5)              # 2 Hz
    ok, hz = slow.check()
    assert not ok and abs(hz - 2.0) < 0.1
    assert slow.violations == 1


def test_rate_monitor_fails_open_when_unknown():
    """No configured rate, or too few samples, must not raise an alarm."""
    assert RateMonitor(expected_hz=None).check() == (True, None)
    m = RateMonitor(expected_hz=10.0)
    m.observe(0.0)
    assert m.check() == (True, None)


def test_rate_monitor_rejects_a_nonsense_expected_rate():
    with pytest.raises(ValueError):
        RateMonitor(expected_hz=-5.0)
