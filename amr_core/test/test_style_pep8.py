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
"""PEP 8 conformance, enforced as a test rather than as a convention."""

import subprocess
from pathlib import Path

import pytest

WORKSPACE = Path(__file__).resolve().parents[3]
SOURCES = [
    "src/amr_core/amr_core",
    "src/amr_core/test",
    "src/amr_bringup/launch",
    "src/amr_bringup/scripts",
    "src/amr_gazebo/scripts",
    "src/amr_gazebo/launch",
    "src/amr_mapping/scripts",
    "src/amr_mapping/launch",
    "src/amr_navigation/scripts",
    "src/amr_navigation/launch",
    "src/amr_safety/scripts",
    "src/amr_safety/launch",
]


def test_custom_nodes_are_pep8_clean():
    paths = [str(WORKSPACE / s) for s in SOURCES if (WORKSPACE / s).is_dir()]
    if not paths:
        pytest.skip("sources not laid out as expected")
    result = subprocess.run(
        ["python3", "-m", "flake8", *paths],
        cwd=str(WORKSPACE), capture_output=True, text=True)
    assert result.returncode == 0, (
        "PEP 8 violations:\n" + (result.stdout or result.stderr))
