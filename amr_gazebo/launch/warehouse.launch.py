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
"""Complete simulation environment in one command."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

T_SPAWN = 8.0
T_OBSTACLES = 14.0


def generate_launch_description():
    pkg = get_package_share_directory("amr_gazebo")
    headless = LaunchConfiguration("headless")
    obstacles = LaunchConfiguration("obstacles")

    world = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg, "launch", "warehouse_world.launch.py")
        ),
        launch_arguments={"headless": headless}.items(),
    )

    spawn_fleet = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg, "launch", "spawn_fleet.launch.py")
        )
    )

    obstacle_field = Node(
        package="amr_gazebo",
        executable="random_walk_obstacle.py",
        name="dynamic_obstacle_field",
        output="screen",
        parameters=[{"use_sim_time": True}],
        condition=IfCondition(obstacles),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "headless", default_value="false",
            description="true runs the Gazebo server only, with no GUI. The "
                        "server has proven far more stable under load than the "
                        "GUI on this machine."),
        DeclareLaunchArgument(
            "obstacles", default_value="true",
            description="false leaves the pedestrians and third-party robots "
                        "parked, which is useful when isolating a navigation "
                        "problem from moving-obstacle interference."),

        LogInfo(msg="[warehouse] starting world + ROS/Gazebo bridge"),
        world,

        TimerAction(period=T_SPAWN, actions=[
            LogInfo(msg="[warehouse] spawning AMR-1 and AMR-2"),
            spawn_fleet,
        ]),

        TimerAction(period=T_OBSTACLES, actions=[
            LogInfo(msg="[warehouse] starting dynamic obstacle field"),
            obstacle_field,
        ]),

        TimerAction(period=T_OBSTACLES + 4.0, actions=[
            LogInfo(msg="[warehouse] environment ready. Ctrl+C stops everything."),
        ]),
    ])
