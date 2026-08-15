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
"""Task 2 in one command: cooperative SLAM, map fusion, and adaptive planning."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    LogInfo,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

T_MAPPING = 30.0
T_NAVIGATION = 70.0
T_RVIZ = 75.0
T_GOALS = 125.0


def _include(package, launch_file, arguments=None):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory(package), "launch", launch_file)
        ),
        launch_arguments=(arguments or {}).items(),
    )


def generate_launch_description():
    headless = LaunchConfiguration("headless")
    goals = LaunchConfiguration("goals")
    rviz = LaunchConfiguration("rviz")

    return LaunchDescription([
        DeclareLaunchArgument("headless", default_value="false",
                              description="true runs Gazebo server-only."),
        DeclareLaunchArgument("goals", default_value="false",
                              description="Dispatch the concurrent navigation "
                                          "goals once the stack is up."),
        DeclareLaunchArgument("perfect_localization", default_value="false",
                              description="SIMULATION AID: anchor TF to Gazebo "
                                          "truth instead of SLAM's correction. "
                                          "SLAM still builds the fused map."),
        DeclareLaunchArgument("safety", default_value="false",
                              description="true when task3's safety/smoothing "
                                          "chain will run. Controls whether the "
                                          "controller publishes cmd_vel_nav "
                                          "(chain present) or cmd_vel directly."),
        DeclareLaunchArgument("rviz", default_value="false",
                              description="Open RViz showing the unified map, "
                                          "both robots' plans, frontiers and "
                                          "ramp regions."),

        LogInfo(msg="[task2] 1/3 world + fleet + dynamic obstacles"),
        _include("amr_bringup", "warehouse_fleet.launch.py",
                 {"headless": headless}),

        TimerAction(period=T_MAPPING, actions=[
            LogInfo(msg="[task2] 2/3 cooperative SLAM + selective mapping + fusion"),
            _include("amr_mapping", "mapping.launch.py"),
        ]),

        TimerAction(period=T_NAVIGATION, actions=[
            LogInfo(msg="[task2] 3/3 Nav2 with ramp-aware global planning"),
            _include("amr_navigation", "navigation.launch.py",
                     {"use_safety_chain": LaunchConfiguration("safety"),
                      "perfect_localization":
                          LaunchConfiguration("perfect_localization")}),
        ]),

        TimerAction(period=T_RVIZ, actions=[
            Node(
                package="rviz2", executable="rviz2", name="rviz2", output="log",
                arguments=["-d", os.path.join(
                    get_package_share_directory("amr_bringup"), "rviz", "fleet.rviz")],
                condition=IfCondition(rviz),
            ),
        ]),

        TimerAction(period=T_GOALS, actions=[
            LogInfo(msg="[task2] dispatching concurrent goals "
                        "(AMR-1 -> Heavy Storage, AMR-2 -> Packing Bay 4)"),
            ExecuteProcess(
                cmd=["ros2", "run", "amr_navigation", "send_concurrent_goals.py"],
                output="screen", condition=IfCondition(goals),
            ),
        ]),

        TimerAction(period=T_GOALS + 5.0, actions=[
            LogInfo(msg="[task2] up. Ctrl+C stops everything."),
        ]),
    ])
