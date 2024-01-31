# Copyright 2021 Tier IV, Inc. All rights reserved.
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

from launch.launch_description import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    # Topic Monitor For Front Lidar PointCloud
    topic_state_monitor_pandar_front_center = ComposableNode(
        package="topic_state_monitor",
        plugin="topic_state_monitor::TopicStateMonitorNode",
        name="topic_state_monitor_pandar_front_center",
        parameters=[
            {
                "topic": "/sensing/lidar/front_center/pointcloud_raw",
                "topic_type": "sensor_msgs/msg/PointCloud2",
                "best_effort": True,
                "diag_name": "sensing_topic_status",
                "warn_rate": 5.0,
                "error_rate": 1.0,
                "timeout": 1.0,
                "window_size": 10,
            },
        ],
        extra_arguments=[{"use_intra_process_comms": True}],
    )

    # Topic Monitor For Concat PointCloud
    topic_state_monitor_top_outlier_filtered = ComposableNode(
        package="topic_state_monitor",
        plugin="topic_state_monitor::TopicStateMonitorNode",
        name="topic_state_monitor_top_outlier_filtered",
        parameters=[
            {
                "topic": "/sensing/lidar/top/pointcloud",
                "topic_type": "sensor_msgs/msg/PointCloud2",
                "best_effort": True,
                "diag_name": "sensing_topic_status",
                "warn_rate": 5.0,
                "error_rate": 1.0,
                "timeout": 1.0,
                "window_size": 10,
            },
        ],
        extra_arguments=[{"use_intra_process_comms": True}],
    )

    topic_state_monitor_pandar_front_center_outlier_filtered = ComposableNode(
        package="topic_state_monitor",
        plugin="topic_state_monitor::TopicStateMonitorNode",
        name="topic_state_monitor_pandar_front_center_outlier_filtered",
        parameters=[
            {
                "topic": "/sensing/lidar/front_center/pointcloud",
                "topic_type": "sensor_msgs/msg/PointCloud2",
                "best_effort": True,
                "diag_name": "sensing_topic_status",
                "warn_rate": 5.0,
                "error_rate": 1.0,
                "timeout": 1.0,
                "window_size": 10,
            },
        ],
        extra_arguments=[{"use_intra_process_comms": True}],
    )

    # Topic Monitor for NoGroundFilter
    topic_state_monitor_rough_no_ground = ComposableNode(
        package="topic_state_monitor",
        plugin="topic_state_monitor::TopicStateMonitorNode",
        name="topic_state_monitor_rough_no_ground",
        parameters=[
            {
                "topic": "/perception/obstacle_segmentation/single_frame/pointcloud",
                "topic_type": "sensor_msgs/msg/PointCloud2",
                "best_effort": True,
                "diag_name": "sensing_topic_status",
                "warn_rate": 5.0,
                "error_rate": 1.0,
                "timeout": 1.0,
                "window_size": 10,
            },
        ],
        extra_arguments=[{"use_intra_process_comms": True}],
    )
    # topic_state_monitor_short_height_no_ground = ComposableNode(
    #     package="topic_state_monitor",
    #     plugin="topic_state_monitor::TopicStateMonitorNode",
    #     name="topic_state_monitor_short_height_no_ground",
    #     parameters=[
    #         {
    #             "topic": "/perception/obstacle_segmentation/plane_fitting/pointcloud",
    #             "topic_type": "sensor_msgs/msg/PointCloud2",
    #             "best_effort": True,
    #             "diag_name": "sensing_topic_status",
    #             "warn_rate": 5.0,
    #             "error_rate": 1.0,
    #             "timeout": 1.0,
    #             "window_size": 10,
    #         },
    #     ],
    #     extra_arguments=[{"use_intra_process_comms": True}],
    # )

    # topic monitor for tamagawa IMU
    topic_state_monitor_imu_data = ComposableNode(
        package="topic_state_monitor",
        plugin="topic_state_monitor::TopicStateMonitorNode",
        name="topic_state_monitor_imu_data",
        parameters=[
            {
                "topic": "/sensing/imu/imu_data",
                "topic_type": "sensor_msgs/msg/Imu",
                "best_effort": False,
                "diag_name": "sensing_topic_status",
                "warn_rate": 5.0,
                "error_rate": 1.0,
                "timeout": 1.0,
                "window_size": 10,
            },
        ],
        extra_arguments=[{"use_intra_process_comms": True}],
    )

    # set container to run all required components in the same process
    container = ComposableNodeContainer(
        name="topic_state_monitor_container",
        namespace="topic_state_monitor",
        package="rclcpp_components",
        executable="component_container",
        composable_node_descriptions=[
            topic_state_monitor_pandar_front_center,
            topic_state_monitor_top_outlier_filtered,
            topic_state_monitor_pandar_front_center_outlier_filtered,
            topic_state_monitor_rough_no_ground,
            # topic_state_monitor_short_height_no_ground,
            topic_state_monitor_imu_data,
        ],
        output="screen",
    )

    return LaunchDescription([container])
