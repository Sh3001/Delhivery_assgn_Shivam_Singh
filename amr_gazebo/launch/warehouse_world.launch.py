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
"""Bring up the Ignition Gazebo (Fortress) warehouse world and the."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    pkg_amr_gazebo = get_package_share_directory("amr_gazebo")
    pkg_ros_gz_sim = get_package_share_directory("ros_gz_sim")

    models_dir = os.path.join(pkg_amr_gazebo, "models")
    existing_resource_path = os.environ.get("GZ_SIM_RESOURCE_PATH", "")
    resource_path = (
        models_dir + os.pathsep + existing_resource_path
        if existing_resource_path
        else models_dir
    )
    set_resource_path = SetEnvironmentVariable("GZ_SIM_RESOURCE_PATH", resource_path)

    world_arg = DeclareLaunchArgument(
        "world",
        default_value=os.path.join(pkg_amr_gazebo, "worlds", "warehouse.sdf"),
        description="Path to the Gazebo world SDF file",
    )
    headless_arg = DeclareLaunchArgument(
        "headless",
        default_value="false",
        description="Run the Gazebo server only, without the GUI client "
                    "(useful in CI/no-display environments)",
    )

    world = LaunchConfiguration("world")
    headless = LaunchConfiguration("headless")

    gz_sim_headless = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, "launch", "gz_sim.launch.py")
        ),
        launch_arguments={"gz_args": [world, " -r -s --headless-rendering"]}.items(),
        condition=IfCondition(headless),
    )

    gz_sim_gui = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, "launch", "gz_sim.launch.py")
        ),
        launch_arguments={"gz_args": [world, " -r"]}.items(),
        condition=UnlessCondition(headless),
    )

    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="amr_fleet_bridge",
        output="screen",
        parameters=[
            {
                "config_file": PathJoinSubstitution(
                    [pkg_amr_gazebo, "config", "bridge.yaml"]
                ),
                "use_sim_time": True,
            }
        ],
    )

    return LaunchDescription(
        [
            set_resource_path,
            world_arg,
            headless_arg,
            gz_sim_headless,
            gz_sim_gui,
            bridge,
        ]
    )
