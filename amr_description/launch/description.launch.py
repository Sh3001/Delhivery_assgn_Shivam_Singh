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
"""Launch robot_state_publisher for a single namespaced AMR."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    robot_name_arg = DeclareLaunchArgument(
        "robot_name", default_value="amr1", description="Namespace / TF prefix for this robot"
    )
    robot_model_arg = DeclareLaunchArgument(
        "robot_model",
        default_value="amr1",
        description="Which xacro entry point to use: amr1 or amr2",
    )
    use_sim_time_arg = DeclareLaunchArgument("use_sim_time", default_value="true")

    robot_name = LaunchConfiguration("robot_name")
    robot_model = LaunchConfiguration("robot_model")
    use_sim_time = LaunchConfiguration("use_sim_time")

    xacro_file = PathJoinSubstitution(
        [FindPackageShare("amr_description"), "urdf", [robot_model, ".urdf.xacro"]]
    )

    robot_description = {
        "robot_description": ParameterValue(
            Command(["xacro ", xacro_file, " robot_name:=", robot_name]),
            value_type=str,
        )
    }

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        namespace=robot_name,
        output="screen",
        parameters=[robot_description, {"use_sim_time": use_sim_time}],
    )
    return LaunchDescription(
        [
            robot_name_arg,
            robot_model_arg,
            use_sim_time_arg,
            robot_state_publisher,
        ]
    )
