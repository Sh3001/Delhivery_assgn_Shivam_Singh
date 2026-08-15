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
"""Cooperative SLAM + map fusion: one slam_toolbox instance per robot, the."""

import os

from amr_core import load_fleet
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


ROBOT_STAGGER_SEC = 10.0

_FLEET = load_fleet()


def _fastdds_udp_only():
    """Force FastDDS onto UDP-only for the nodes this file launches."""
    profile = os.path.join(
        get_package_share_directory("amr_bringup"), "config", "fastdds_udp_only.xml")
    return SetEnvironmentVariable("FASTRTPS_DEFAULT_PROFILES_FILE", profile)


def _peer_filters(use_sim_time):
    """One peer scan filter per robot, started with SLAM.

    SLAM must not map peer robots. When two robots spawn close together each
    one's SLAM sees the other, bakes it into the fused map as permanent
    structure, and the robot then stands inside a mapped obstacle - its own
    costmap cell goes lethal and the planner refuses to start. The filters
    used to launch with Nav2, forty seconds after SLAM, which was far too late.
    """
    from amr_core import load_fleet
    return [
        Node(
            package="amr_navigation",
            executable="peer_scan_filter.py",
            name=f"{robot.name}_peer_scan_filter",
            output="screen",
            parameters=[{
                "robot_name": robot.name,
                "input_topic": f"/{robot.name}/scan_validated",
                "output_topic": f"/{robot.name}/scan_no_peers",
                "use_sim_time": use_sim_time,
            }],
        )
        for robot in load_fleet().robots
    ]


def generate_launch_description():
    pkg_amr_mapping = get_package_share_directory("amr_mapping")

    use_sim_time_arg = DeclareLaunchArgument("use_sim_time", default_value="true")
    use_sim_time = LaunchConfiguration("use_sim_time")

    scan_fixer_amr1 = Node(
        package="amr_gazebo",
        executable="sensor_bsp_node.py",
        name="sensor_bsp_node",
        namespace="amr1",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time, "robot_name": "amr1"}],
    )
    scan_fixer_amr2 = Node(
        package="amr_gazebo",
        executable="sensor_bsp_node.py",
        name="sensor_bsp_node",
        namespace="amr2",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time, "robot_name": "amr2"}],
    )

    ekf_amr1 = Node(
        package="robot_localization",
        executable="ekf_node",
        name="amr1_ekf_filter_node",
        output="screen",
        parameters=[
            os.path.join(pkg_amr_mapping, "config", "ekf_amr1.yaml"),
            {"use_sim_time": use_sim_time},
        ],
        remappings=[
            ("odometry/filtered", "/amr1/odometry/filtered"),
            ("set_pose", "/amr1/set_pose"),
            ("/diagnostics", "/amr1/diagnostics"),
        ],
    )
    ekf_amr2 = Node(
        package="robot_localization",
        executable="ekf_node",
        name="amr2_ekf_filter_node",
        output="screen",
        parameters=[
            os.path.join(pkg_amr_mapping, "config", "ekf_amr2.yaml"),
            {"use_sim_time": use_sim_time},
        ],
        remappings=[
            ("odometry/filtered", "/amr2/odometry/filtered"),
            ("set_pose", "/amr2/set_pose"),
            ("/diagnostics", "/amr2/diagnostics"),
        ],
    )

    slam_amr1 = Node(
        package="slam_toolbox",
        executable="async_slam_toolbox_node",
        name="amr1_slam_toolbox",
        output="screen",
        parameters=[
            os.path.join(pkg_amr_mapping, "config", "slam_toolbox_amr1.yaml"),
            {
                "use_sim_time": use_sim_time,
                "scan_topic": "/amr1/scan_no_peers",
                "map_frame": "amr1/map",
                "odom_frame": "amr1/odom",
                "base_frame": "amr1/base_footprint",
            },
        ],
        remappings=[
            ("/map", "/amr1/map"),
            ("/map_metadata", "/amr1/map_metadata"),
            ("/slam_toolbox/graph_visualization", "/amr1/slam_toolbox/graph_visualization"),
            ("/slam_toolbox/scan_visualization", "/amr1/slam_toolbox/scan_visualization"),
        ],
    )

    slam_amr2 = Node(
        package="slam_toolbox",
        executable="async_slam_toolbox_node",
        name="amr2_slam_toolbox",
        output="screen",
        parameters=[
            os.path.join(pkg_amr_mapping, "config", "slam_toolbox_amr2.yaml"),
            {
                "use_sim_time": use_sim_time,
                "scan_topic": "/amr2/scan_no_peers",
                "map_frame": "amr2/map",
                "odom_frame": "amr2/odom",
                "base_frame": "amr2/base_footprint",
            },
        ],
        remappings=[
            ("/map", "/amr2/map"),
            ("/map_metadata", "/amr2/map_metadata"),
            ("/slam_toolbox/graph_visualization", "/amr2/slam_toolbox/graph_visualization"),
            ("/slam_toolbox/scan_visualization", "/amr2/slam_toolbox/scan_visualization"),
        ],
    )

    selective_filter_amr1 = Node(
        package="amr_mapping",
        executable="selective_map_filter.py",
        name="selective_map_filter",
        namespace="amr1",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
    )

    map_merge = Node(
        package="amr_mapping",
        executable="map_merge_node.py",
        name="map_merge_node",
        output="screen",
        parameters=[
            {
                "use_sim_time": use_sim_time,
                **{f"{r.name}_spawn_{axis}": value
                   for r in _FLEET.robots
                   for axis, value in (("x", r.x), ("y", r.y), ("yaw", r.yaw))},
                **{f"global_{k}": v for k, v in _FLEET.map.items()
                   if k in ("x_min", "x_max", "y_min", "y_max")},
                "resolution": _FLEET.map.get("resolution", 0.1),
            }
        ],
    )

    return LaunchDescription(
        [
            _fastdds_udp_only(),
            use_sim_time_arg,
            *_peer_filters(use_sim_time),
            scan_fixer_amr1,
            ekf_amr1,
            slam_amr1,
            selective_filter_amr1,
            TimerAction(
                period=ROBOT_STAGGER_SEC,
                actions=[scan_fixer_amr2, ekf_amr2, slam_amr2],
            ),
            TimerAction(period=2.0 * ROBOT_STAGGER_SEC, actions=[map_merge]),
        ]
    )
