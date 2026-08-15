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
"""Namespaced launch for a 10+ robot fleet - the scalability demonstration."""

import os

from amr_core import load_fleet
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _default_fleet():
    return os.path.join(
        get_package_share_directory("amr_core"), "config", "fleet_10.yaml")


def _spawn_nodes(context):
    """Build one namespaced component set per configured robot."""
    fleet_path = LaunchConfiguration("fleet").perform(context) or _default_fleet()
    fleet = load_fleet(fleet_path)

    actions = [LogInfo(
        msg=f"[fleet] {len(fleet.robots)} robots from {os.path.basename(fleet_path)}: "
            f"{', '.join(r.name for r in fleet.robots)}")]

    for robot in fleet.robots:
        actions.append(
            Node(
                package="amr_gazebo",
                executable="sensor_bsp_node.py",
                name="sensor_bsp_node",
                namespace=robot.name,
                output="screen",
                parameters=[{
                    "robot_name": robot.name,
                    "fleet_config": fleet_path,
                    "use_sim_time": False,
                    "health_report_period_sec": 10.0,
                }],
            )
        )
    actions.append(LogInfo(
        msg="[fleet] all robots up. `ros2 node list` shows one BSP validator "
            "per namespace; no source file names any of them."))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "fleet", default_value="",
            description="Fleet YAML to launch. Defaults to amr_core's "
                        "fleet_10.yaml. Any file with more robots works "
                        "without a code change."),
        OpaqueFunction(function=_spawn_nodes),
    ])
