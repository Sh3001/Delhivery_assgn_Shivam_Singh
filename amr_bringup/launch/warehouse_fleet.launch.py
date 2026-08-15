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
"""Single entry point for the whole simulation environment."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    Shutdown,
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


def _running_simulators():
    """PIDs of Ignition processes already running for this user."""
    pids = []
    for entry in os.listdir("/proc"):
        if not entry.isdigit():
            continue
        pid = int(entry)
        if pid == os.getpid():
            continue
        try:
            with open(f"/proc/{entry}/cmdline", "rb") as handle:
                argv = handle.read().split(b"\x00")
            if not argv or not argv[0]:
                continue
            joined = b" ".join(a for a in argv if a).decode(errors="replace")
            tokens = joined.split()
            if not tokens or os.path.basename(tokens[0]) != "ign":
                continue
            if os.stat(f"/proc/{entry}").st_uid != os.getuid():
                continue
        except (OSError, IndexError):
            continue        # process exited, or not ours to inspect
        pids.append(pid)
    return sorted(pids)


def _preflight():
    """Refuse to start a second simulator on top of a running one."""
    existing = _running_simulators()
    if not existing:
        return []
    pids = " ".join(str(p) for p in existing)
    return [
        LogInfo(msg="=" * 72),
        LogInfo(msg=f"[bringup] ABORTING: Ignition is already running (PID(s): {pids})."),
        LogInfo(msg="[bringup] Starting a second simulator makes every object "
                    "flicker, because two"),
        LogInfo(msg="[bringup] servers publish a pose for each model and the "
                    "viewer alternates between them."),
        LogInfo(msg="[bringup]"),
        LogInfo(msg="[bringup] Stop the old one first:   "
                    "ros2 run amr_bringup stop_stack.py"),
        LogInfo(msg=f"[bringup] or by PID:               kill -9 {pids}"),
        LogInfo(msg="=" * 72),
        Shutdown(reason="an Ignition simulator is already running"),
    ]


def generate_launch_description():
    blocked = _preflight()
    if blocked:
        return LaunchDescription(blocked)

    pkg_gazebo = get_package_share_directory("amr_gazebo")
    headless = LaunchConfiguration("headless")
    obstacles = LaunchConfiguration("obstacles")

    world = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo, "launch", "warehouse_world.launch.py")
        ),
        launch_arguments={"headless": headless}.items(),
    )

    spawn_fleet = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo, "launch", "spawn_fleet.launch.py")
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
                        "parked, which helps when isolating a problem from "
                        "moving-obstacle interference."),

        LogInfo(msg="[bringup] 1/3 warehouse world + ROS/Gazebo bridge"),
        world,

        TimerAction(period=T_SPAWN, actions=[
            LogInfo(msg="[bringup] 2/3 spawning AMR-1 and AMR-2"),
            spawn_fleet,
        ]),

        TimerAction(period=T_OBSTACLES, actions=[
            LogInfo(msg="[bringup] 3/3 dynamic obstacle field"),
            obstacle_field,
        ]),

        TimerAction(period=T_OBSTACLES + 4.0, actions=[
            LogInfo(msg="[bringup] environment ready. Ctrl+C stops everything."),
        ]),
    ])
