# Copyright 2023 Tier IV, Inc. All rights reserved.
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

from ctypes import ArgumentError
import os
from pathlib import Path

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
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterFile
import yaml


def get_lidar_make(sensor_name):
    if sensor_name[:6].lower() == "pandar":
        return "Hesai"
    elif sensor_name[:3].lower() in ["vlp", "vls"]:
        return "Velodyne"
    raise ArgumentError("Unrecognized sensor model")


def get_vehicle_info(context):
    # TODO(TIER IV): Use Parameter Substitution after we drop Galactic support
    # https://github.com/ros2/launch_ros/blob/master/launch_ros/launch_ros/substitutions/parameter.py
    gp = context.launch_configurations.get("ros_params", {})
    if not gp:
        gp = dict(context.launch_configurations.get("global_params", {}))
    p = {}
    p["vehicle_length"] = gp["front_overhang"] + gp["wheel_base"] + gp["rear_overhang"]
    p["vehicle_width"] = gp["wheel_tread"] + gp["left_overhang"] + gp["right_overhang"]
    p["min_longitudinal_offset"] = -gp["rear_overhang"]
    p["max_longitudinal_offset"] = gp["front_overhang"] + gp["wheel_base"]
    p["min_lateral_offset"] = -(gp["wheel_tread"] / 2.0 + gp["right_overhang"])
    p["max_lateral_offset"] = gp["wheel_tread"] / 2.0 + gp["left_overhang"]
    p["min_height_offset"] = 0.0
    p["max_height_offset"] = gp["vehicle_height"]
    return p


def launch_setup(context, *args, **kwargs):
    def load_composable_node_param(param_path):
        with open(LaunchConfiguration(param_path).perform(context), "r") as f:
            return yaml.safe_load(f)["/**"]["ros__parameters"]

    def create_parameter_dict(*args):
        result = {}
        for x in args:
            result[x] = LaunchConfiguration(x)
        return result

    # Model and make
    sensor_model = LaunchConfiguration("sensor_model").perform(context)
    sensor_make = get_lidar_make(sensor_model)

    # Configuration file containing sensor model's default parameters
    
    sensor_config = LaunchConfiguration("config_file").perform(context)
    if (sensor_config == ""):
        sensor_config = (
            Path(get_package_share_directory("nebula_ros"))
            / "config"
            / "lidar"
            / sensor_make.lower()
            / (sensor_model + ".param.yaml")
        )
    else:
        sensor_config = Path(sensor_config)

    assert sensor_config.exists(), "Sensor configuration not found: {}".format(
        sensor_config.absolute().as_posix()
    )

    nodes = []

    nodes.append(
        ComposableNode(
            package="glog_component",
            plugin="GlogComponent",
            name="glog_component",
        )
    )

    nodes.append(
        ComposableNode(
            package="nebula_ros",
            plugin=sensor_make + "RosWrapper",
            name=sensor_make.lower() + "_ros_wrapper_node",
            parameters=[
                ParameterFile(sensor_config, allow_substs=True),
                {
                    "sensor_model": sensor_model,
                    "launch_hw": LaunchConfiguration("launch_driver"),
                    **create_parameter_dict(
                        "host_ip",
                        "sensor_ip",
                        "data_port",
                        "gnss_port",
                        "packet_mtu_size",
                        "setup_sensor",
                        "frame_id",
                        "diag_span",
                        "min_range",
                        "max_range",
                        "cloud_min_angle",
                        "cloud_max_angle",
                        "scan_phase",
                        "rotation_speed",
                        "return_mode",
                        "ptp_profile",
                        "ptp_domain",
                        "ptp_transport_type",
                        "ptp_switch_type",
                        "retry_hw",
                        "dual_return_distance_threshold",
                    ),
                },
            ],
            remappings=[
                ("aw_points", "pointcloud_raw"),
                ("aw_points_ex", "pointcloud_raw_ex"),
            ],
            extra_arguments=[{"use_intra_process_comms": LaunchConfiguration("use_intra_process")}],
        )
    )

    cropbox_parameters = create_parameter_dict("input_frame", "output_frame")
    cropbox_parameters["negative"] = True

    vehicle_info = get_vehicle_info(context)
    cropbox_parameters["min_x"] = vehicle_info["min_longitudinal_offset"]
    cropbox_parameters["max_x"] = vehicle_info["max_longitudinal_offset"]
    cropbox_parameters["min_y"] = vehicle_info["min_lateral_offset"]
    cropbox_parameters["max_y"] = vehicle_info["max_lateral_offset"]
    cropbox_parameters["min_z"] = vehicle_info["min_height_offset"]
    cropbox_parameters["max_z"] = vehicle_info["max_height_offset"]

    nodes.append(
        ComposableNode(
            package="pointcloud_preprocessor",
            plugin="pointcloud_preprocessor::CropBoxFilterComponent",
            name="crop_box_filter_self",
            remappings=[
                ("input", "pointcloud_raw_ex"),
                ("output", "self_cropped/pointcloud_ex"),
            ],
            parameters=[cropbox_parameters],
            extra_arguments=[{"use_intra_process_comms": LaunchConfiguration("use_intra_process")}],
        )
    )

    mirror_info = load_composable_node_param("vehicle_mirror_param_file")
    cropbox_parameters["min_x"] = mirror_info["min_longitudinal_offset"]
    cropbox_parameters["max_x"] = mirror_info["max_longitudinal_offset"]
    cropbox_parameters["min_y"] = mirror_info["min_lateral_offset"]
    cropbox_parameters["max_y"] = mirror_info["max_lateral_offset"]
    cropbox_parameters["min_z"] = mirror_info["min_height_offset"]
    cropbox_parameters["max_z"] = mirror_info["max_height_offset"]

    nodes.append(
        ComposableNode(
            package="pointcloud_preprocessor",
            plugin="pointcloud_preprocessor::CropBoxFilterComponent",
            name="crop_box_filter_mirror",
            remappings=[
                ("input", "self_cropped/pointcloud_ex"),
                ("output", "mirror_cropped/pointcloud_ex"),
            ],
            parameters=[cropbox_parameters],
            extra_arguments=[{"use_intra_process_comms": LaunchConfiguration("use_intra_process")}],
        )
    )

    nodes.append(
        ComposableNode(
            package="pointcloud_preprocessor",
            plugin="pointcloud_preprocessor::DistortionCorrectorComponent",
            name="distortion_corrector_node",
            remappings=[
                ("~/input/twist", "/sensing/vehicle_velocity_converter/twist_with_covariance"),
                ("~/input/imu", "/sensing/imu/imu_data"),
                ("~/input/pointcloud", "mirror_cropped/pointcloud_ex"),
                ("~/output/pointcloud", "rectified/pointcloud_ex"),
            ],
            extra_arguments=[{"use_intra_process_comms": LaunchConfiguration("use_intra_process")}],
        )
    )

    # Ring Outlier Filter is the last component in the pipeline, so control the output frame here
    if LaunchConfiguration("output_as_sensor_frame").perform(context):
        ring_outlier_filter_parameters = {"output_frame": LaunchConfiguration("frame_id")}
    else:
        ring_outlier_filter_parameters = {
            "output_frame": ""
        }  # keep the output frame as the input frame
    nodes.append(
        ComposableNode(
            package="pointcloud_preprocessor",
            plugin="pointcloud_preprocessor::RingOutlierFilterComponent",
            name="ring_outlier_filter",
            remappings=[
                ("input", "rectified/pointcloud_ex"),
                ("output", "pointcloud_before_sync"),
            ],
            parameters=[ring_outlier_filter_parameters],
            extra_arguments=[{"use_intra_process_comms": LaunchConfiguration("use_intra_process")}],
        )
    )

    # set container to run all required components in the same process
    container = ComposableNodeContainer(
        name=LaunchConfiguration("container_name"),
        namespace="pointcloud_preprocessor",
        package="rclcpp_components",
        executable=LaunchConfiguration("container_executable"),
        composable_node_descriptions=nodes,
        output="both",
    )

    blockage_diag_component = ComposableNode(
        package="pointcloud_preprocessor",
        plugin="pointcloud_preprocessor::BlockageDiagComponent",
        name="blockage_diag",
        remappings=[
            ("input", "pointcloud_raw_ex"),
            ("output", "blockage_diag/pointcloud"),
        ],
        parameters=[
            {
                "angle_range": [
                    float(context.perform_substitution(LaunchConfiguration("cloud_min_angle"))),
                    float(context.perform_substitution(LaunchConfiguration("cloud_max_angle"))),
                ],
                "horizontal_ring_id": LaunchConfiguration("horizontal_ring_id"),
                "vertical_bins": LaunchConfiguration("vertical_bins"),
                "is_channel_order_top2down": LaunchConfiguration("is_channel_order_top2down"),
                "max_distance_range": LaunchConfiguration("max_range"),
                "horizontal_resolution": LaunchConfiguration("horizontal_resolution"),
            }
        ]
        + [load_composable_node_param("blockage_diagnostics_param_file")],
        extra_arguments=[{"use_intra_process_comms": LaunchConfiguration("use_intra_process")}],
    )

    blockage_diag_loader = LoadComposableNodes(
        composable_node_descriptions=[blockage_diag_component],
        target_container=container,
        condition=IfCondition(LaunchConfiguration("enable_blockage_diag")),
    )

    return [container, blockage_diag_loader]


def generate_launch_description():
    launch_arguments = []

    def add_launch_arg(name: str, default_value=None, description=None):
        # a default_value of None is equivalent to not passing that kwarg at all
        launch_arguments.append(
            DeclareLaunchArgument(name, default_value=default_value, description=description)
        )

    add_launch_arg("sensor_model", description="sensor model name")
    add_launch_arg("config_file", "", description="sensor configuration file")
    add_launch_arg("launch_driver", "True", "whether to connect to a sensor or to listen to packet replays instead")
    add_launch_arg("setup_sensor", "True", "configure sensor")
    add_launch_arg("retry_hw", "false", "retry hw")
    add_launch_arg("sensor_ip", "192.168.1.201", "device ip address")
    add_launch_arg("host_ip", "255.255.255.255", "host ip address")
    add_launch_arg("scan_phase", "0.0")
    add_launch_arg("base_frame", "base_link", "base frame id")
    add_launch_arg("min_range", "0.3", "minimum view range for Velodyne sensors")
    add_launch_arg("max_range", "300.0", "maximum view range for Velodyne sensors")
    add_launch_arg("cloud_min_angle", "0", "minimum view angle setting on device")
    add_launch_arg("cloud_max_angle", "360", "maximum view angle setting on device")
    add_launch_arg("data_port", "2368", "device data port number")
    add_launch_arg("gnss_port", "2380", "device gnss port number")
    add_launch_arg("packet_mtu_size", "1500", "packet mtu size")
    add_launch_arg("rotation_speed", "600", "rotational frequency")
    add_launch_arg("dual_return_distance_threshold", "0.1", "dual return distance threshold")
    add_launch_arg("frame_id", "lidar", "frame id")
    add_launch_arg("input_frame", LaunchConfiguration("base_frame"), "use for cropbox")
    add_launch_arg("output_frame", LaunchConfiguration("base_frame"), "use for cropbox")
    add_launch_arg(
        "vehicle_mirror_param_file", description="path to the file of vehicle mirror position yaml"
    )
    add_launch_arg("use_multithread", "False", "use multithread")
    add_launch_arg("use_intra_process", "False", "use ROS 2 component container communication")
    add_launch_arg("container_name", "nebula_node_container")
    add_launch_arg("ptp_profile", "1588v2")
    add_launch_arg("ptp_transport_type", "L2")
    add_launch_arg("ptp_switch_type", "TSN")
    add_launch_arg("ptp_domain", "0")
    add_launch_arg("output_as_sensor_frame", "True", "output final pointcloud in sensor frame")
    add_launch_arg("diag_span", "1000", "")
    add_launch_arg("output_as_sensor_frame", "True", "output final pointcloud in sensor frame")

    add_launch_arg("enable_blockage_diag", "true")
    add_launch_arg("horizontal_ring_id", "64")
    add_launch_arg("vertical_bins", "128")
    add_launch_arg("is_channel_order_top2down", "true")
    add_launch_arg("horizontal_resolution", "0.4")
    add_launch_arg(
        "blockage_diagnostics_param_file",
        [FindPackageShare("common_sensor_launch"), "/config/blockage_diagnostics_param_file.yaml"],
    )

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
