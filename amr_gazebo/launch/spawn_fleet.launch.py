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
"""Spawn the heterogeneous AMR fleet (AMR-1 + AMR-2) into the already-running."""

import os

from amr_core import load_fleet
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

SPAWN_HEIGHT_BY_MODEL = {"heavy_tugger": 0.15, "light_scout": 0.10}


def generate_launch_description():
    pkg_amr_description = get_package_share_directory("amr_description")

    fleet = load_fleet()

    actions = []
    for robot in fleet.robots:
        name = model = robot.name
        x, y, yaw = robot.spawn
        z = SPAWN_HEIGHT_BY_MODEL.get(robot.model_name, 0.15)

        description = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_amr_description, "launch", "description.launch.py")
            ),
            launch_arguments={
                "robot_name": name,
                "robot_model": model,
                "use_sim_time": "true",
            }.items(),
        )

        spawn = Node(
            package="ros_gz_sim",
            executable="create",
            name=f"spawn_{name}",
            output="screen",
            arguments=[
                "-topic", f"/{name}/robot_description",
                "-name", name,
                "-x", str(x), "-y", str(y), "-z", str(z),
                "-Y", str(yaw),
            ],
        )

        actions.extend([description, spawn])

    return LaunchDescription(actions)
