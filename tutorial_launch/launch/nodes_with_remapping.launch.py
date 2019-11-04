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

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import launch_ros.actions


def generate_launch_description():

    # Substitution that can access launch configuration variables.
    namespace = LaunchConfiguration('ns_variable')

    # Action that declares a new launch argument.
    declare_namespace_cmd = DeclareLaunchArgument(
        'ns_variable',                      # variable name
        default_value='new_ns',             # default value
        description='Top-level namespace')  # description

    # Action that executes a ROS node.
    publisher_node = launch_ros.actions.Node(
        package='tutorial_publisher',   # package name
        node_executable='talker',       # executable file name
        output='log',                   # output is selected of log, screen, both
        node_name='publisher',          # change node name
        node_namespace=namespace,       # add namespace
        remappings=[((namespace, '/topic'), (namespace, '/new_topic'))]  # topic remapping
    )

    # Action that executes a ROS node.
    subscriber_node = launch_ros.actions.Node(
        package='tutorial_listener',    # package name
        node_executable='listener',     # executable file name
        output='log',                   # output is selected of log, screen, both
        node_name='subscriber',         # change node name
        node_namespace=namespace,       # add namespace
        remappings=[((namespace, '/topic'), (namespace, '/new_topic'))]  # topic remapping
    )

    # Description of a launch-able system.
    ld = LaunchDescription()

    # Add an action to the LaunchDescription.
    ld.add_action(declare_namespace_cmd)
    ld.add_action(publisher_node)
    ld.add_action(subscriber_node)

    return ld
