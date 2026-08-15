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
"""Task 3 in one command: local control, conflict handling and safety override."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

T_SAFETY = 110.0


def generate_launch_description():
    headless = LaunchConfiguration("headless")
    goals = LaunchConfiguration("goals")
    rviz = LaunchConfiguration("rviz")
    pkg = get_package_share_directory("amr_bringup")

    return LaunchDescription([
        DeclareLaunchArgument("headless", default_value="false"),
        DeclareLaunchArgument("goals", default_value="false"),
        DeclareLaunchArgument("rviz", default_value="false"),
        DeclareLaunchArgument("perfect_localization", default_value="false"),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(pkg, "launch", "task2.launch.py")),
            launch_arguments={"headless": headless, "goals": goals,
                              "rviz": rviz, "safety": "true",
                              "perfect_localization": LaunchConfiguration(
                                  "perfect_localization")}.items(),
        ),

        TimerAction(period=T_SAFETY, actions=[
            LogInfo(msg="[task3] motion smoothing + traffic control + safety override"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(get_package_share_directory("amr_safety"),
                                 "launch", "safety_stack.launch.py"))),
        ]),
    ])
