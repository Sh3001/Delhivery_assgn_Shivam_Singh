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
"""Loads the fleet configuration and resolves each robot against its model."""

import os

import yaml
from ament_index_python.packages import get_package_share_directory


class FleetConfigError(RuntimeError):
    """Raised when the configuration is unusable."""


class RobotConfig:
    """One robot instance, with its model's properties flattened onto it."""

    def __init__(self, name, model_name, spawn, model):
        self.name = name
        self.model_name = model_name
        self.x, self.y, self.yaw = spawn
        for key, value in model.items():
            setattr(self, key, value)

    @property
    def spawn(self):
        return (self.x, self.y, self.yaw)

    def __repr__(self):
        return f"RobotConfig({self.name!r}, model={self.model_name!r}, spawn={self.spawn})"


class FleetConfig:
    def __init__(self, robots, policy, map_bounds, global_frame, ramps=None):
        self.robots = robots
        self.ramps = ramps or []
        self.policy = policy
        self.map = map_bounds
        self.global_frame = global_frame

    @property
    def names(self):
        return [r.name for r in self.robots]

    def robot(self, name):
        for r in self.robots:
            if r.name == name:
                return r
        raise FleetConfigError(
            f"no robot named {name!r} in the fleet; have {self.names}")


def _require(mapping, key, context):
    if key not in mapping:
        raise FleetConfigError(f"{context}: missing required key {key!r}")
    return mapping[key]


def load_fleet(fleet_path=None):
    """Read fleet.yaml plus its referenced model library and resolve the two."""
    if fleet_path is None:
        fleet_path = os.path.join(
            get_package_share_directory("amr_core"), "config", "fleet.yaml")

    with open(fleet_path) as handle:
        root = yaml.safe_load(handle) or {}
    fleet = _require(root, "fleet", fleet_path)

    library_path = os.path.join(
        os.path.dirname(fleet_path), _require(fleet, "model_library", fleet_path))
    with open(library_path) as handle:
        models = yaml.safe_load(handle) or {}

    robots = []
    seen_names = set()
    seen_priorities = {}
    for entry in _require(fleet, "robots", fleet_path):
        name = _require(entry, "name", "robot entry")
        model_name = _require(entry, "model", f"robot {name}")
        if name in seen_names:
            raise FleetConfigError(f"duplicate robot name {name!r}")
        seen_names.add(name)
        if model_name not in models:
            raise FleetConfigError(
                f"robot {name!r} references unknown model {model_name!r}; "
                f"library has {sorted(models)}")

        model = dict(models[model_name])
        reserved = {"name", "model", "x", "y", "yaw"}
        overrides = {k: v for k, v in entry.items() if k not in reserved}
        unknown = sorted(set(overrides) - set(model))
        if unknown:
            raise FleetConfigError(
                f"robot {name!r} sets {unknown} which model {model_name!r} "
                f"does not define; known fields are {sorted(model)}")
        model.update(overrides)

        priority = model.get("yield_priority")
        if priority in seen_priorities:
            raise FleetConfigError(
                f"robots {seen_priorities[priority]!r} and {name!r} share "
                f"yield_priority {priority}; conflict resolution would be ambiguous")
        seen_priorities[priority] = name

        robots.append(RobotConfig(
            name, model_name,
            (float(entry.get("x", 0.0)), float(entry.get("y", 0.0)),
             float(entry.get("yaw", 0.0))),
            model))

    if not robots:
        raise FleetConfigError(f"{fleet_path}: fleet has no robots")

    return FleetConfig(
        robots=robots,
        policy=fleet.get("policy", {}),
        map_bounds=fleet.get("map", {}),
        global_frame=fleet.get("global_frame", "map"),
        ramps=fleet.get("ramps", []),
    )
