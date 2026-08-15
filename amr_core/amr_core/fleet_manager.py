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
"""Fleet manager: one reusable component set per robot, built from config."""

from amr_core.conflict import TrafficPolicy
from amr_core.fleet_config import load_fleet
from amr_core.motion_smoothing import MotionSmoother
from amr_core.safety import SafetyMonitor
from amr_core.sensor_bsp import ImuValidator, LidarValidator, RateMonitor


class RobotInstance:
    """Everything one robot needs, assembled from its configuration."""

    def __init__(self, config, expect_frames=True):
        self.config = config
        self.name = config.name
        self.model_name = config.model_name

        self.namespace = f"/{self.name}"

        self.lidar_frame = f"{self.name}/lidar_link"
        self.imu_frame = f"{self.name}/imu_link"

        try:
            token = self.name if expect_frames else None
            self.imu_validator = ImuValidator(
                config,
                expected_frame=self.imu_frame if expect_frames else None,
                frame_token=token)
            self.lidar_validator = LidarValidator(
                config,
                expected_frame=self.lidar_frame if expect_frames else None,
                frame_token=token)
            self.imu_rate = RateMonitor(getattr(config, "imu_rate_hz", None))
            self.lidar_rate = RateMonitor(getattr(config, "lidar_rate_hz", None))
            self.motion_smoother = MotionSmoother(config)
            self.safety_monitor = SafetyMonitor(config)
        except (ValueError, AttributeError) as exc:
            raise ValueError(
                f"robot '{self.name}' (model '{self.model_name}') is "
                f"misconfigured: {exc}") from exc

    def topic(self, name):
        """Absolute topic for this robot. Single place namespacing is decided."""
        return f"{self.namespace}/{name.lstrip('/')}"

    @property
    def interfaces(self):
        """The raw -> validated mapping this robot presents."""
        return {
            "raw": {"lidar": self.topic("scan"), "imu": self.topic("imu")},
            "validated": {"lidar": self.topic("scan_validated"),
                          "imu": self.topic("imu_validated")},
            "diagnostics": self.topic("sensor_health"),
        }

    def diagnostics(self):
        """Flat, publishable snapshot of this robot's validation state."""
        imu, lidar = self.imu_validator.counts, self.lidar_validator.counts
        return {
            "robot": self.name,
            "model": self.model_name,
            "imu_total": imu["total"],
            "imu_rejected": imu["reject"],
            "imu_angular_velocity_violations": imu["angular_velocity_violations"],
            "imu_axis_violations": dict(self.imu_validator.axis_violations),
            "imu_stale": imu["stale"],
            "lidar_total": lidar["total"],
            "lidar_rejected": lidar["reject"],
            "lidar_out_of_range_readings": lidar["out_of_range_readings"],
            "lidar_stale": lidar["stale"],
            "imu_rate_violations": self.imu_rate.violations,
            "lidar_rate_violations": self.lidar_rate.violations,
        }

    def __repr__(self):
        return f"RobotInstance({self.name}, model={self.model_name})"


class FleetManager:
    """Builds and owns a `RobotInstance` per configured robot."""

    def __init__(self, fleet=None, expect_frames=True):
        self.fleet = fleet if fleet is not None else load_fleet()
        self.robots = {}
        for config in self.fleet.robots:
            if config.name in self.robots:
                raise ValueError(f"duplicate robot name '{config.name}'")
            self.robots[config.name] = RobotInstance(
                config, expect_frames=expect_frames)

        policy = self.fleet.policy
        self.traffic_policy = TrafficPolicy(
            margin=float(policy.get("conflict_margin", 0.35)),
            clear_margin=float(policy.get("clear_margin", 1.0)),
            slow_speed_scale=float(policy.get("slow_speed_scale", 0.35)),
        )

    @property
    def names(self):
        return list(self.robots)

    @property
    def size(self):
        return len(self.robots)

    def robot(self, name):
        try:
            return self.robots[name]
        except KeyError:
            raise KeyError(
                f"no robot '{name}' in this fleet; have {self.names}") from None

    def namespaces(self):
        return [r.namespace for r in self.robots.values()]

    def all_topics(self):
        """Every interface the fleet exposes - used to prove no collisions."""
        topics = []
        for r in self.robots.values():
            ifaces = r.interfaces
            topics.extend(ifaces["raw"].values())
            topics.extend(ifaces["validated"].values())
            topics.append(ifaces["diagnostics"])
        return topics

    def diagnostics(self):
        return {name: r.diagnostics() for name, r in self.robots.items()}

    def __len__(self):
        return len(self.robots)

    def __iter__(self):
        return iter(self.robots.values())

    def __repr__(self):
        return f"FleetManager({self.size} robots: {', '.join(self.names)})"
