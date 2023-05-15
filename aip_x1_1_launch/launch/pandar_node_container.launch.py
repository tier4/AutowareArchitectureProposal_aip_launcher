# Copyright 2020 Tier IV, Inc. All rights reserved.
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
import launch
from launch.actions import DeclareLaunchArgument
from launch.actions import OpaqueFunction
from launch.actions import SetLaunchConfiguration
from launch.conditions import IfCondition
from launch.conditions import UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.actions import LoadComposableNodes
from launch_ros.actions import Node
from launch_ros.descriptions import ComposableNode
import yaml


def get_pandar_monitor_info():
    path = os.path.join(
        get_package_share_directory("pandar_monitor"),
        "config",
        "pandar_monitor.param.yaml",
    )
    with open(path, "r") as f:
        p = yaml.safe_load(f)["/**"]["ros__parameters"]
    return p


def launch_setup(context, *args, **kwargs):
    def create_parameter_dict(*args):
        result = {}
        for x in args:
            result[x] = LaunchConfiguration(x)
        return result

    monitor_node = Node(
        executable="pandar_monitor_node",
        package="pandar_monitor",
        name="pandar_monitor",
        parameters=[
            {
                "ip_address": LaunchConfiguration("device_ip"),
            }
        ]
        + [get_pandar_monitor_info()],
        condition=IfCondition(LaunchConfiguration("launch_driver")),
        output="screen",
    )

    driver_component = ComposableNode(
        package="pandar_driver",
        plugin="pandar_driver::PandarDriver",
        name="pandar_driver",
        parameters=[
            {
                **create_parameter_dict(
                    "pcap", "device_ip", "lidar_port", "gps_port", "scan_phase", "model", "frame_id"
                )
            }
        ],
    )

    pointcloud_component = ComposableNode(
        package="pandar_pointcloud",
        plugin="pandar_pointcloud::PandarCloud",
        name="pandar_cloud",
        parameters=[
            {
                **create_parameter_dict(
                    "model",
                    "scan_phase",
                    "angle_range",
                    "distance_range",
                    "device_ip",
                    "calibration",
                    "return_mode",
                )
            }
        ],
        remappings=[("pandar_points", "pointcloud_raw"), ("pandar_points_ex", "pointcloud_raw_ex")],
        extra_arguments=[{"use_intra_process_comms": LaunchConfiguration("use_intra_process")}],
    )

    undistort_component = ComposableNode(
        package="pointcloud_preprocessor",
        plugin="pointcloud_preprocessor::DistortionCorrectorComponent",
        name="distortion_corrector_node",
        remappings=[
            ("~/input/velocity_report", "/vehicle/status/velocity_status"),
            ("~/input/pointcloud", "pointcloud_raw_ex"),
            ("~/output/pointcloud", "rectified/pointcloud_ex"),
        ],
        extra_arguments=[{"use_intra_process_comms": LaunchConfiguration("use_intra_process")}],
    )

    ring_outlier_filter_component = ComposableNode(
        package="pointcloud_preprocessor",
        plugin="pointcloud_preprocessor::RingOutlierFilterComponent",
        name="ring_outlier_filter",
        remappings=[
            ("input", "rectified/pointcloud_ex"),
            ("output", "outlier_filtered/pointcloud"),
        ],
        extra_arguments=[{"use_intra_process_comms": LaunchConfiguration("use_intra_process")}],
    )

    container = ComposableNodeContainer(
        name="pandar_node_container",
        namespace="pointcloud_preprocessor",
        package="rclcpp_components",
        executable=LaunchConfiguration("container_executable"),
        composable_node_descriptions=[
            pointcloud_component,
            undistort_component,
            ring_outlier_filter_component,
        ],
    )

    driver_loader = LoadComposableNodes(
        composable_node_descriptions=[driver_component],
        target_container=container,
        condition=launch.conditions.IfCondition(LaunchConfiguration("launch_driver")),
    )

    return [
        container,
        driver_loader,
        monitor_node,
    ]


def generate_launch_description():
    launch_arguments = []

    def add_launch_arg(name: str, default_value=None):
        launch_arguments.append(DeclareLaunchArgument(name, default_value=default_value))

    add_launch_arg("model")
    add_launch_arg("launch_driver", "true")
    add_launch_arg("calibration", "")
    add_launch_arg("device_ip", "192.168.1.201")
    add_launch_arg("scan_phase", "0.0")
    add_launch_arg("angle_range", "[0.0, 360.0]")
    add_launch_arg("distance_range", "[0.05, 200.0]")
    add_launch_arg("return_mode", "Dual")
    add_launch_arg("container_name", "pandar_composable_node_container")
    add_launch_arg("pcap", "")
    add_launch_arg("lidar_port", "2321")
    add_launch_arg("gps_port", "10121")
    add_launch_arg("frame_id", "pandar")
    add_launch_arg("use_multithread", "False")
    add_launch_arg("use_intra_process", "False")

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
