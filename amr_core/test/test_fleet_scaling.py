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
"""Namespacing and fleet-scalability tests (Sections 8, 10, 12, 14)."""

import os

import pytest
from amr_core import FleetManager, RobotInstance, load_fleet
from amr_core.fleet_config import FleetConfigError
from amr_core.motion_smoothing import MotionSmoother
from amr_core.safety import SafetyMonitor
from amr_core.sensor_bsp import ImuValidator, LidarValidator

CONFIG_DIR = os.path.join(os.path.dirname(__file__), "..", "config")
FLEET_2 = os.path.abspath(os.path.join(CONFIG_DIR, "fleet.yaml"))
FLEET_10 = os.path.abspath(os.path.join(CONFIG_DIR, "fleet_10.yaml"))


def fleet_of(path):
    return FleetManager(load_fleet(path))


def test_ten_robot_fleet_initialises_from_configuration_alone():
    """Section 15, Test 3: ten robots, no per-robot source code."""
    fleet = fleet_of(FLEET_10)
    assert fleet.size == 10
    assert fleet.names == [f"amr{i}" for i in range(1, 11)]


def test_every_robot_uses_the_same_reusable_classes():
    """No AMR3Controller anywhere - one class, ten configurations."""
    for robot in fleet_of(FLEET_10):
        assert type(robot) is RobotInstance
        assert type(robot.imu_validator) is ImuValidator
        assert type(robot.lidar_validator) is LidarValidator
        assert type(robot.motion_smoother) is MotionSmoother
        assert type(robot.safety_monitor) is SafetyMonitor


def test_scaling_up_does_not_change_the_two_robot_behaviour():
    """amr1 and amr2 must behave identically in both fleets."""
    small = fleet_of(FLEET_2)
    large = fleet_of(FLEET_10)
    for name in ("amr1", "amr2"):
        a, b = small.robot(name), large.robot(name)
        assert a.model_name == b.model_name
        assert (a.imu_validator.angular_velocity_limits
                == b.imu_validator.angular_velocity_limits)
        assert a.safety_monitor.d_min == b.safety_monitor.d_min


def test_robots_may_mix_models_freely():
    fleet = fleet_of(FLEET_10)
    models = {r.model_name for r in fleet}
    assert models == {"heavy_tugger", "light_scout"}, models


def test_per_robot_overrides_beat_the_model_default():
    """Ten robots cannot share two models without this."""
    fleet = fleet_of(FLEET_10)
    assert fleet.robot("amr3").model_name == "light_scout"
    assert fleet.robot("amr3").config.yield_priority == 45
    assert fleet.robot("amr2").config.yield_priority == 50


def test_yield_priorities_are_unique_across_the_whole_fleet():
    """Ambiguous priority would make the yield protocol undecidable."""
    priorities = [r.config.yield_priority for r in fleet_of(FLEET_10)]
    assert len(set(priorities)) == len(priorities), priorities


def test_namespaces_are_distinct():
    fleet = fleet_of(FLEET_10)
    namespaces = fleet.namespaces()
    assert len(set(namespaces)) == 10
    assert "/amr7" in namespaces


def test_no_topic_collides_across_ten_robots():
    """The core namespacing guarantee, asserted over every interface."""
    topics = fleet_of(FLEET_10).all_topics()
    assert len(topics) == len(set(topics)), "two robots share a topic"
    assert len(topics) == 10 * 5      # raw x2, validated x2, diagnostics


def test_every_topic_is_under_its_own_robot_namespace():
    for robot in fleet_of(FLEET_10):
        ifaces = robot.interfaces
        every = (list(ifaces["raw"].values()) + list(ifaces["validated"].values())
                 + [ifaces["diagnostics"]])
        for topic in every:
            assert topic.startswith(f"/{robot.name}/"), topic


def test_amr1_and_amr10_do_not_collide_on_prefix():
    """'/amr1' is a string prefix of '/amr10' - a real source of cross-talk."""
    fleet = fleet_of(FLEET_10)
    one = set(fleet.robot("amr1").interfaces["validated"].values())
    ten = set(fleet.robot("amr10").interfaces["validated"].values())
    assert not (one & ten)
    for topic in ten:
        assert not topic.startswith("/amr1/")


def test_raw_and_validated_interfaces_are_separate():
    for robot in fleet_of(FLEET_10):
        ifaces = robot.interfaces
        assert set(ifaces["raw"]) == set(ifaces["validated"]) == {"lidar", "imu"}
        for sensor in ("lidar", "imu"):
            assert ifaces["raw"][sensor] != ifaces["validated"][sensor]
            assert "validated" in ifaces["validated"][sensor]
            assert "validated" not in ifaces["raw"][sensor]


def test_diagnostics_identify_the_robot():
    fleet = fleet_of(FLEET_10)
    diag = fleet.diagnostics()
    assert set(diag) == set(fleet.names)
    assert diag["amr6"]["robot"] == "amr6"
    for required in ("imu_rejected", "imu_angular_velocity_violations",
                     "imu_axis_violations", "imu_stale", "lidar_rejected"):
        assert required in diag["amr6"]


def test_unknown_robot_is_reported_with_the_available_names():
    with pytest.raises(KeyError, match="amr99"):
        fleet_of(FLEET_2).robot("amr99")


def test_a_typo_in_an_override_is_rejected_not_ignored(tmp_path):
    """A silently ignored override would hand back the model default."""
    fleet_yaml = tmp_path / "bad.yaml"
    fleet_yaml.write_text(
        "fleet:\n"
        f"  model_library: '{os.path.join(CONFIG_DIR, 'robot_models.yaml')}'\n"
        "  robots:\n"
        "    - {name: 'amrX', model: 'light_scout', x: 0.0, y: 0.0, "
        "yeild_priority: 7}\n")
    with pytest.raises(FleetConfigError, match="yeild_priority"):
        load_fleet(str(fleet_yaml))
