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
"""The whole system, one command."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

READY_CHECK_START = 75.0


def generate_launch_description():
    pkg = get_package_share_directory("amr_bringup")

    return LaunchDescription([
        DeclareLaunchArgument("headless", default_value="false",
                              description="true runs Gazebo server-only."),
        DeclareLaunchArgument("rviz", default_value="true",
                              description="Open RViz with the fleet view."),
        DeclareLaunchArgument("obstacles", default_value="true",
                              description="Run the pedestrian and third-party "
                                          "robot field."),
        DeclareLaunchArgument("perfect_localization", default_value="true",
                              description="Simulation aid: anchor TF to Gazebo "
                                          "ground truth so a given goal is "
                                          "reachable. false uses SLAM alone."),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(pkg, "launch", "task3.launch.py")),
            launch_arguments={
                "headless": LaunchConfiguration("headless"),
                "rviz": LaunchConfiguration("rviz"),
                "obstacles": LaunchConfiguration("obstacles"),
                "perfect_localization": LaunchConfiguration("perfect_localization"),
                "goals": "false",
            }.items(),
        ),

        TimerAction(period=READY_CHECK_START, actions=[
            Node(
                package="amr_bringup",
                executable="fleet_ready.py",
                name="fleet_ready",
                output="screen",
            ),
        ]),
    ])
