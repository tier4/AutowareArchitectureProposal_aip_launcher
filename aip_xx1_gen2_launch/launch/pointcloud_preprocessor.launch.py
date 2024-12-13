# Copyright 2024 TIER IV, Inc. All rights reserved.
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


import launch
from launch.actions import DeclareLaunchArgument
from launch.actions import OpaqueFunction
from launch.actions import SetLaunchConfiguration
from launch.conditions import IfCondition
from launch.conditions import UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LoadComposableNodes
from launch_ros.descriptions import ComposableNode


def launch_setup(context, *args, **kwargs):
    # set concat filter as a component
    concat_component = ComposableNode(
        package="pointcloud_preprocessor",
        plugin="pointcloud_preprocessor::PointCloudConcatenateDataSynchronizerComponent",
        name="concatenate_data",
        remappings=[
            ("~/input/twist", "/sensing/vehicle_velocity_converter/twist_with_covariance"),
            ("output", "concatenated/pointcloud"),
        ],
        parameters=[
            {
                "input_topics": LaunchConfiguration("input_topics"),
                "output_frame": LaunchConfiguration("base_frame"),
                "input_offset": LaunchConfiguration(
                    "input_offset"
                ),  # each sensor will wait 60, 70, 70, 70ms
                "timeout_sec": LaunchConfiguration("timeout_sec"),  # set shorter than 100ms
                "input_twist_topic_type": LaunchConfiguration("input_twist_topic_type"),
                "publish_synchronized_pointcloud": LaunchConfiguration(
                    "publish_synchronized_pointcloud"
                ),
            }
        ],
        extra_arguments=[{"use_intra_process_comms": LaunchConfiguration("use_intra_process")}],
    )

    # load concat or passthrough filter
    concat_loader = LoadComposableNodes(
        composable_node_descriptions=[concat_component],
        target_container=LaunchConfiguration("pointcloud_container_name"),
        condition=IfCondition(LaunchConfiguration("use_concat_filter")),
    )

    return [concat_loader]


def generate_launch_description():
    launch_arguments = []

    def add_launch_arg(name: str, default_value=None):
        launch_arguments.append(DeclareLaunchArgument(name, default_value=default_value))

    add_launch_arg("base_frame", "base_link")
    add_launch_arg("use_multithread", "False")
    add_launch_arg("use_intra_process", "False")
    add_launch_arg("pointcloud_container_name", "pointcloud_container")
    add_launch_arg("individual_container_name", "concatenate_container")
    add_launch_arg(
        "input_topics",
        "[/sensing/lidar/top/pointcloud, /sensing/lidar/left/pointcloud, /sensing/lidar/right/pointcloud, /sensing/lidar/rear/pointcloud]",
    )
    add_launch_arg("input_offset", "[0.035, 0.025, 0.025, 0.025]")
    add_launch_arg("timeout_sec", "0.095")
    add_launch_arg("input_twist_topic_type", "twist")
    add_launch_arg("publish_synchronized_pointcloud", "False")

    set_container_executable = SetLaunchConfiguration(
        "container_executable",
        "component_container",
        condition=UnlessCondition(LaunchConfiguration("use_multithread")),
    )

    set_container_mt_executable = SetLaunchConfiguration(
        "container_executable",
        "component_container_mt",
        condition=IfCondition(LaunchConfiguration("use_multithread")),
    )

    return launch.LaunchDescription(
        launch_arguments
        + [set_container_executable, set_container_mt_executable]
        + [OpaqueFunction(function=launch_setup)]
    )
