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


from copy import deepcopy
import os
from typing import Any
from typing import List

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import GroupAction
from launch.actions import IncludeLaunchDescription
from launch.actions import OpaqueFunction
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import PushRosNamespace
import yaml


def join_list_of_arguments(arguments: List[Any]) -> str:
    """Join a list of arguments into a string, used by Include Launch Description.

    Example:
        join_list_of_arguments([1,2,3]) -> "[1, 2, 3]"
    """
    return f"[{', '.join([str(arg) for arg in arguments])}]"


def generate_launch_dictionary():
    path_dictionary = {
        "hesai_OT128": AnyLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("common_sensor_launch"),
                "launch",
                "hesai_OT128.launch.xml",
            )
        ),
        "hesai_XT32": AnyLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("common_sensor_launch"),
                "launch",
                "hesai_XT32.launch.xml",
            )
        ),
        "velodyne_VLS128": AnyLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("common_sensor_launch"),
                "launch",
                "velodyne_VLS128.launch.xml",
            )
        ),
        "velodyne_VLP16": AnyLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("common_sensor_launch"),
                "launch",
                "velodyne_VLP16.launch.xml",
            )
        ),
        "livox_horizon": AnyLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("common_sensor_launch"),
                "launch",
                "livox_horizon.launch.py",
            )
        ),
    }
    return path_dictionary


def load_sub_launches_from_yaml(context, *args, **kwargs):
    def load_yaml(yaml_file_path):
        with open(LaunchConfiguration(yaml_file_path).perform(context), "r") as f:
            return yaml.safe_load(f)

    config = load_yaml("config_file")

    path_dictionary = generate_launch_dictionary()

    base_parameters = {}
    base_parameters["host_ip"] = LaunchConfiguration("host_ip").perform(context)
    base_parameters["vehicle_mirror_param_file"] = LaunchConfiguration(
        "vehicle_mirror_param_file"
    ).perform(context)
    base_parameters["launch_driver"] = LaunchConfiguration("launch_driver").perform(context)
    base_parameters["launch_hw_monitor"] = LaunchConfiguration("launch_hw_monitor").perform(context)
    base_parameters["vehicle_id"] = LaunchConfiguration("vehicle_id").perform(context)
    base_parameters["pointcloud_container_name"] = LaunchConfiguration(
        "pointcloud_container_name"
    ).perform(context)
    base_parameters["enable_blockage_diag"] = LaunchConfiguration("enable_blockage_diag").perform(
        context
    )
    base_parameters["return_mode"] = LaunchConfiguration("return_mode").perform(context)

    sub_launch_actions = []
    for launch in config["launches"]:
        launch_parameters = deepcopy(base_parameters)
        launch_parameters.update(launch["parameters"])  # dict
        launch_parameter_list_tuple = [(str(k), str(v)) for k, v in launch_parameters.items()]
        sub_launch_action = GroupAction(
            [
                PushRosNamespace(launch["namespace"]),
                IncludeLaunchDescription(
                    deepcopy(path_dictionary[launch["sensor_type"]]),
                    launch_arguments=launch_parameter_list_tuple,
                ),
            ]
        )
        sub_launch_actions.append(sub_launch_action)

    processor_dict = config["preprocessor"]
    sub_launch_actions.append(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory("aip_xx1_gen2_launch"),
                    "launch",
                    "pointcloud_preprocessor.launch.py",
                )
            ),
            launch_arguments=[
                ("base_frame", "base_link"),
                ("use_multithread", "true"),
                ("use_intra_process", "true"),
                ("use_pointcloud_container", LaunchConfiguration("use_pointcloud_container")),
                ("pointcloud_container_name", LaunchConfiguration("pointcloud_container_name")),
                ("input_topics", join_list_of_arguments(processor_dict["input_topics"])),
                ("input_offset", join_list_of_arguments(processor_dict["input_offset"])),
                ("timeout_sec", str(processor_dict["timeout_sec"])),
                ("input_twist_topic_type", str(processor_dict["input_twist_topic_type"])),
                (
                    "publish_synchronized_pointcloud",
                    str(processor_dict["publish_synchronized_pointcloud"]),
                ),
            ],
        )
    )
    return [
        GroupAction([PushRosNamespace("lidar"), *sub_launch_actions]),
    ]


def generate_launch_description():
    # Define launch arguments
    launch_arguments = []
    config_file_arg = DeclareLaunchArgument(
        "config_file",
        default_value=os.path.join(
            get_package_share_directory("aip_xx1_gen2_launch"), "config", "lidar_gen2.yaml"
        ),
        description="Path to the configuration file",
    )
    launch_arguments.append(config_file_arg)

    def add_launch_arg(name: str, default_value=None, **kwargs):
        launch_arguments.append(DeclareLaunchArgument(name, default_value=default_value, **kwargs))

    add_launch_arg("launch_driver", "true")
    add_launch_arg("launch_hw_monitor", "true", description="launch hardware monitor")
    add_launch_arg("host_ip", "192.168.1.10")
    add_launch_arg("use_concat_filter", "true")
    add_launch_arg(
        "vehicle_id",
        default_value=EnvironmentVariable("VEHICLE_ID", default_value="default"),
    )
    add_launch_arg("vehicle_mirror_param_file")
    add_launch_arg("use_pointcloud_container", "false", description="launch pointcloud container")
    add_launch_arg("pointcloud_container_name", "pointcloud_container")
    add_launch_arg("enable_blockage_diag", "false")
    add_launch_arg("return_mode", "Dual")

    # Create launch description with the config_file argument
    ld = LaunchDescription(launch_arguments)
    # Add sub-launch files dynamically based on the YAML configuration
    ld.add_action(OpaqueFunction(function=load_sub_launches_from_yaml))

    return ld
