# Copyright 2019 Open Source Robotics Foundation, Inc.
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

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    # Get the launch directory
    package_dir = get_package_share_directory('tutorial_launch')
    launch_dir = os.path.join(package_dir, 'launch')

    launch_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                launch_dir,
                'nodes.launch.py'
            )
        )
    )

    ld = LaunchDescription()

    ld.add_action(launch_description)

    return ld
