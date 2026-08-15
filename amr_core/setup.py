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
from setuptools import setup

package_name = "amr_core"

setup(
    name=package_name,
    version="1.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config",
         ["config/robot_models.yaml", "config/fleet.yaml",
          "config/fleet_10.yaml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Shivam",
    maintainer_email="s1.shivam3001@gmail.com",
    description="Fleet model library and configuration loader.",
    license="Apache-2.0",
)
