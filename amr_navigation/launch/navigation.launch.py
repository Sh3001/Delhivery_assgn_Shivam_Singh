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
"""Nav2 bringup for both robots against the cooperatively-built merged map."""

import os

from amr_core import load_fleet
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.actions import DeclareLaunchArgument, GroupAction, TimerAction, SetEnvironmentVariable
from launch.substitutions import PythonExpression, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import RewrittenYaml

PARAMS_BY_ROBOT = {"amr1": "nav2_params_amr1.yaml", "amr2": "nav2_params_amr2.yaml"}

ROBOT_STAGGER_SEC = 12.0

LIFECYCLE_NODES = [
    "controller_server",
    "smoother_server",
    "planner_server",
    "behavior_server",
    "bt_navigator",
    "waypoint_follower",
]


def _make_robot_group(pkg_amr_navigation, robot, use_sim_time, autostart,
                      use_safety_chain, perfect_localization):
    name = robot["name"]
    raw_params_file = os.path.join(pkg_amr_navigation, "config", robot["params"])
    x, y, yaw = robot["spawn"]

    params_file = ParameterFile(
        RewrittenYaml(
            source_file=raw_params_file,
            root_key=name,
            param_rewrites={"use_sim_time": use_sim_time, "autostart": autostart},
            convert_types=True,
        ),
        allow_substs=True,
    )

    map_to_local_map = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name=f"{name}_map_frame_link",
        condition=UnlessCondition(perfect_localization),
        arguments=[
            "--x", str(x), "--y", str(y), "--z", "0",
            "--yaw", str(yaw), "--pitch", "0", "--roll", "0",
            "--frame-id", "map",
            "--child-frame-id", f"{name}/map",
        ],
    )

    nodes = [
        Node(
            package="nav2_controller",
            executable="controller_server",
            name="controller_server",
            namespace=name,
            output="screen",
            parameters=[params_file],
            remappings=[("cmd_vel", PythonExpression([
                "'cmd_vel_nav' if '", use_safety_chain, "' == 'true' else 'cmd_vel'"
            ]))],
        ),
        Node(
            package="nav2_smoother",
            executable="smoother_server",
            name="smoother_server",
            namespace=name,
            output="screen",
            parameters=[params_file],
        ),
        Node(
            package="nav2_planner",
            executable="planner_server",
            name="planner_server",
            namespace=name,
            output="screen",
            parameters=[params_file],
        ),
        Node(
            package="nav2_behaviors",
            executable="behavior_server",
            name="behavior_server",
            namespace=name,
            output="screen",
            parameters=[params_file],
        ),
        Node(
            package="nav2_bt_navigator",
            executable="bt_navigator",
            name="bt_navigator",
            namespace=name,
            output="screen",
            parameters=[params_file],
        ),
        Node(
            package="nav2_waypoint_follower",
            executable="waypoint_follower",
            name="waypoint_follower",
            namespace=name,
            output="screen",
            parameters=[params_file],
        ),
        Node(
            package="nav2_lifecycle_manager",
            executable="lifecycle_manager",
            name="lifecycle_manager_navigation",
            namespace=name,
            output="screen",
            parameters=[
                {
                    "use_sim_time": use_sim_time,
                    "autostart": autostart,
                    "node_names": LIFECYCLE_NODES,
                    "bond_timeout": 60.0,
                    "attempt_respawn_reconnection": True,
                    "bond_respawn_max_duration": 30.0,
                }
            ],
        ),
    ]

    return GroupAction(actions=[map_to_local_map] + nodes)


def _fastdds_udp_only():
    """Force FastDDS onto UDP-only for the nodes this file launches."""
    profile = os.path.join(
        get_package_share_directory("amr_bringup"), "config", "fastdds_udp_only.xml")
    return SetEnvironmentVariable("FASTRTPS_DEFAULT_PROFILES_FILE", profile)


def generate_launch_description():
    pkg_amr_navigation = get_package_share_directory("amr_navigation")

    use_sim_time_arg = DeclareLaunchArgument("use_sim_time", default_value="true")
    autostart_arg = DeclareLaunchArgument("autostart", default_value="true")
    use_safety_chain_arg = DeclareLaunchArgument(
        "use_safety_chain", default_value="false",
        description="true when amr_safety's smoother/override chain is running: "
                    "the controller then publishes cmd_vel_nav and the safety "
                    "override owns cmd_vel. false makes the controller publish "
                    "cmd_vel directly, which is required when navigation runs "
                    "without the Section 3 stack.")
    use_safety_chain = LaunchConfiguration("use_safety_chain")
    perfect_localization_arg = DeclareLaunchArgument(
        "perfect_localization", default_value="false",
        description="SIMULATION AID. true anchors the TF chain to Gazebo "
                    "ground truth instead of SLAM's correction, so navigation "
                    "plans for a robot that is where it believes it is. SLAM "
                    "still runs and still builds the fused map. Off by default.")
    perfect_localization = LaunchConfiguration("perfect_localization")
    use_sim_time = LaunchConfiguration("use_sim_time")
    autostart = LaunchConfiguration("autostart")

    fleet = load_fleet()
    groups = []
    for index, robot in enumerate(
            [{"name": r.name, "params": PARAMS_BY_ROBOT[r.name], "spawn": r.spawn}
             for r in fleet.robots]):
        group = _make_robot_group(
            pkg_amr_navigation, robot, use_sim_time, autostart,
            use_safety_chain, perfect_localization)
        if index == 0:
            groups.append(group)
        else:
            groups.append(TimerAction(period=float(index) * ROBOT_STAGGER_SEC, actions=[group]))

    ramp_markers = Node(
        package="amr_navigation",
        executable="ramp_region_markers.py",
        name="ramp_region_markers",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
    )

    return LaunchDescription(
        [
            _fastdds_udp_only(),
            use_sim_time_arg,
            autostart_arg,
            use_safety_chain_arg,
            perfect_localization_arg,
            Node(
                package="amr_mapping",
                executable="ground_truth_localization.py",
                name="ground_truth_localization",
                output="screen",
                parameters=[{"use_sim_time": use_sim_time,
                             "robots": [r.name for r in load_fleet().robots]}],
                condition=IfCondition(perfect_localization),
            ),
            ramp_markers,
            *groups,
        ]
    )
