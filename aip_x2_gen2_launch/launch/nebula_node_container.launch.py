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

import launch
from launch.actions import DeclareLaunchArgument
from launch.actions import OpaqueFunction
from launch.actions import SetLaunchConfiguration

# from launch.conditions import LaunchConfigurationNotEquals
from launch.conditions import IfCondition
from launch.conditions import LaunchConfigurationEquals
from launch.conditions import LaunchConfigurationNotEquals
from launch.conditions import UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.actions import LoadComposableNodes
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare
import yaml


def get_lidar_make(sensor_name):
    if sensor_name[:6].lower() == "pandar":
        return "Hesai", ".csv"
    elif sensor_name[:3].lower() in ["hdl", "vlp", "vls"]:
        return "Velodyne", ".yaml"
    return "unrecognized_sensor_model"


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

    def str2vector(string):
        return [float(x) for x in string.strip("[]").split(",")]

    # Model and make
    sensor_model = LaunchConfiguration("sensor_model").perform(context)
    sensor_make, sensor_extension = get_lidar_make(sensor_model)

    glog_component = ComposableNode(
        package="glog_component",
        plugin="GlogComponent",
        name="glog_component",
    )

    nebula_component = ComposableNode(
        package="nebula_ros",
        plugin=sensor_make + "RosWrapper",
        name=sensor_make.lower() + "_ros_wrapper_node",
        parameters=[
            {
                "sensor_model": sensor_model,
                **create_parameter_dict(
                    "host_ip",
                    "sensor_ip",
                    "multicast_ip",
                    "data_port",
                    "return_mode",
                    "min_range",
                    "max_range",
                    "frame_id",
                    "sync_angle",
                    "cut_angle",
                    "dual_return_distance_threshold",
                    "rotation_speed",
                    "cloud_min_angle",
                    "cloud_max_angle",
                    "gnss_port",
                    "packet_mtu_size",
                    "setup_sensor",
                    "ptp_profile",
                    "ptp_transport_type",
                    "ptp_switch_type",
                    "ptp_domain",
                    "diag_span",
                    "calibration_file",
                    "launch_hw",
                ),
                "retry_hw": True,
            },
        ]
        + [load_composable_node_param("point_filters_param_file")],
        remappings=[
            # ("aw_points", "pointcloud_raw"),
            ("pandar_points", "pointcloud_raw_ex"),
        ],
        extra_arguments=[{"use_intra_process_comms": LaunchConfiguration("use_intra_process")}],
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

    self_crop_component = ComposableNode(
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

    undistort_component = ComposableNode(
        package="pointcloud_preprocessor",
        plugin="pointcloud_preprocessor::DistortionCorrectorComponent",
        name="distortion_corrector_node",
        remappings=[
            ("~/input/twist", "/sensing/vehicle_velocity_converter/twist_with_covariance"),
            ("~/input/imu", "/sensing/imu/imu_data"),
            ("~/input/pointcloud", "self_cropped/pointcloud_ex"),
            ("~/output/pointcloud", "rectified/pointcloud_ex"),
        ],
        parameters=[load_composable_node_param("distortion_corrector_node_param_file")],
        extra_arguments=[{"use_intra_process_comms": LaunchConfiguration("use_intra_process")}],
    )

    ring_outlier_filter_component = ComposableNode(
        package="pointcloud_preprocessor",
        plugin="pointcloud_preprocessor::RingOutlierFilterComponent",
        name="ring_outlier_filter",
        remappings=[
            ("input", "rectified/pointcloud_ex"),
            ("output", "pointcloud"),
        ],
        extra_arguments=[{"use_intra_process_comms": LaunchConfiguration("use_intra_process")}],
    )

    dual_return_filter_component = ComposableNode(
        package="pointcloud_preprocessor",
        plugin="pointcloud_preprocessor::DualReturnOutlierFilterComponent",
        name="dual_return_filter",
        remappings=[
            ("input", "rectified/pointcloud_ex"),
            ("output", "pointcloud"),
        ],
        parameters=[
            {
                "vertical_bins": LaunchConfiguration("vertical_bins"),
                "min_azimuth_deg": LaunchConfiguration("min_azimuth_deg"),
                "max_azimuth_deg": LaunchConfiguration("max_azimuth_deg"),
            }
        ]
        + [load_composable_node_param("dual_return_filter_param_file")],
        extra_arguments=[{"use_intra_process_comms": LaunchConfiguration("use_intra_process")}],
    )

    distance_range = str2vector(context.perform_substitution(LaunchConfiguration("distance_range")))
    blockage_diag_component = ComposableNode(
        package="pointcloud_preprocessor",
        plugin="pointcloud_preprocessor::BlockageDiagComponent",
        name="blockage_return_diag",
        remappings=[
            ("input", "pointcloud_raw_ex"),
            ("output", "blockage_diag/pointcloud"),
        ],
        parameters=[
            {
                "angle_range": LaunchConfiguration("blockage_range"),
                "horizontal_ring_id": LaunchConfiguration("horizontal_ring_id"),
                "vertical_bins": LaunchConfiguration("vertical_bins"),
                "is_channel_order_top2down": LaunchConfiguration("is_channel_order_top2down"),
                "max_distance_range": distance_range[1],
                "horizontal_resolution": LaunchConfiguration("horizontal_resolution"),
            }
        ]
        + [load_composable_node_param("blockage_diagnostics_param_file")],
        extra_arguments=[{"use_intra_process_comms": LaunchConfiguration("use_intra_process")}],
    )

    container = ComposableNodeContainer(
        name="nebula_node_container",
        namespace="pointcloud_preprocessor",
        package="rclcpp_components",
        executable=LaunchConfiguration("container_executable"),
        composable_node_descriptions=[
            glog_component,
            nebula_component,
            self_crop_component,
            undistort_component,
        ],
    )

    ring_outlier_filter_loader = LoadComposableNodes(
        composable_node_descriptions=[ring_outlier_filter_component],
        target_container=container,
        condition=LaunchConfigurationNotEquals("return_mode", "Dual"),
    )

    dual_return_filter_loader = LoadComposableNodes(
        composable_node_descriptions=[dual_return_filter_component],
        target_container=container,
        condition=LaunchConfigurationEquals("return_mode", "Dual"),
    )

    blockage_diag_loader = LoadComposableNodes(
        composable_node_descriptions=[blockage_diag_component],
        target_container=container,
        condition=launch.conditions.IfCondition(LaunchConfiguration("enable_blockage_diag")),
    )

    return [
        container,
        ring_outlier_filter_loader,
        dual_return_filter_loader,
        blockage_diag_loader,
    ]


def generate_launch_description():
    launch_arguments = []

    def add_launch_arg(name: str, default_value=None, description=None):
        # a default_value of None is equivalent to not passing that kwarg at all
        launch_arguments.append(
            DeclareLaunchArgument(name, default_value=default_value, description=description)
        )

    add_launch_arg("sensor_model", description="sensor model name")
    add_launch_arg("config_file", "", description="sensor configuration file")
    add_launch_arg("launch_hw", "True", "do launch driver")
    add_launch_arg("setup_sensor", "True", "configure sensor")
    add_launch_arg("sensor_ip", "192.168.1.201", "device ip address")
    add_launch_arg(
        "multicast_ip",
        "",
        "the multicast group the sensor shall broadcast to. leave empty to disable multicast",
    )
    add_launch_arg("host_ip", "255.255.255.255", "host ip address")
    add_launch_arg("sync_angle", "0")
    add_launch_arg("cut_angle", "0.0")
    # add_launch_arg("point_filters", "{}", "point filter definitions in JSON format")
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
    add_launch_arg("diag_span", "1000")
    add_launch_arg("use_multithread", "False", "use multithread")
    add_launch_arg("use_intra_process", "False", "use ROS 2 component container communication")
    add_launch_arg("container_name", "nebula_node_container")

    add_launch_arg("dual_return_filter_param_file")
    add_launch_arg(
        "blockage_diagnostics_param_file",
        [FindPackageShare("common_sensor_launch"), "/config/blockage_diagnostics.param.yaml"],
    )
    add_launch_arg(
        "distortion_corrector_node_param_file",
        [FindPackageShare("common_sensor_launch"), "/config/distortion_corrector_node.param.yaml"],
    )
    add_launch_arg("vertical_bins", "128")
    add_launch_arg("horizontal_ring_id", "12")
    add_launch_arg("blockage_range", "[270.0, 90.0]")

    add_launch_arg("min_azimuth_deg", "135.0")
    add_launch_arg("max_azimuth_deg", "225.0")
    add_launch_arg("enable_blockage_diag", "true")
    add_launch_arg("point_filters_param_file")

    add_launch_arg("calibration_file", "")

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
