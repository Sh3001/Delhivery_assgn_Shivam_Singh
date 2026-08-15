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
"""Section 3: local execution + safety layer, one per-robot pair plus one."""

import os

from ament_index_python.packages import get_package_share_directory
from amr_core import load_fleet
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


_FLEET = load_fleet()


def _motion_smoother(pkg, name, use_sim_time):
    return Node(
        package="amr_safety",
        executable="motion_smoother_node.py",
        name=f"{name}_motion_smoother_node",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time, "robot_name": name}],
        remappings=[
            ("cmd_vel_nav", f"/{name}/cmd_vel_nav"),
            ("payload_loaded", f"/{name}/payload_loaded"),
            ("cmd_vel_smoothed_local", f"/{name}/cmd_vel_smoothed_local"),
            ("speed_scale", f"/{name}/speed_scale"),
        ],
    )


def _safety_override(pkg, name, use_sim_time):
    return Node(
        package="amr_safety",
        executable="safety_override_node.py",
        name=f"{name}_safety_override_node",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time, "robot_name": name}],
        remappings=[
            ("cmd_vel_smoothed_local", f"/{name}/cmd_vel_smoothed_local"),
            ("odometry/filtered", f"/{name}/odometry/filtered"),
            ("scan_fixed", f"/{name}/scan_validated"),
            ("yield_stop", f"/{name}/yield_stop"),
            ("cmd_vel", f"/{name}/cmd_vel"),
            ("safety_stop_active", f"/{name}/safety_stop_active"),
        ],
    )


def _fastdds_udp_only():
    """Force FastDDS onto UDP-only for the nodes this file launches."""
    profile = os.path.join(
        get_package_share_directory("amr_bringup"), "config", "fastdds_udp_only.xml")
    return SetEnvironmentVariable("FASTRTPS_DEFAULT_PROFILES_FILE", profile)


def generate_launch_description():
    pkg = get_package_share_directory("amr_safety")

    use_sim_time_arg = DeclareLaunchArgument("use_sim_time", default_value="true")
    use_sim_time = LaunchConfiguration("use_sim_time")

    traffic_control = Node(
        package="amr_safety",
        executable="traffic_control_node.py",
        name="traffic_control_node",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
    )

    return LaunchDescription(
        [
            _fastdds_udp_only(),
            use_sim_time_arg,
            *[_motion_smoother(pkg, name, use_sim_time) for name in _FLEET.names],
            *[_safety_override(pkg, name, use_sim_time) for name in _FLEET.names],
            traffic_control,
        ]
    )
