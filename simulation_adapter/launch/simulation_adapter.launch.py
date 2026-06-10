#!/usr/bin/env python3

# Copyright Institute for Automotive Engineering (ika), RWTH Aachen University
# SPDX-License-Identifier: Apache-2.0

import os

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetParameter


def generate_launch_description():
    """Generate the simulation adapter launch description."""
    remappable_topics = [
        DeclareLaunchArgument(
            "input_map_info_topic", default_value="~/input_map_info", description="input topic for simulation map info"
        ),
        DeclareLaunchArgument(
            "input_ego_data_topic", default_value="~/input_ego_data", description="input topic for simulation ego data"
        ),
        DeclareLaunchArgument(
            "input_object_list_topic",
            default_value="~/input_object_list",
            description="input topic for simulation object lists",
        ),
        DeclareLaunchArgument(
            "input_trajectory_topic",
            default_value="~/input_trajectory",
            description="input topic for planned trajectories",
        ),
        DeclareLaunchArgument(
            "output_ego_data_topic", default_value="~/ego_data", description="output topic for transformed ego data"
        ),
        DeclareLaunchArgument(
            "output_object_list_topic",
            default_value="~/object_list",
            description="output topic for transformed object lists",
        ),
        DeclareLaunchArgument(
            "output_object_list_fixed_topic",
            default_value="~/object_list_fixed",
            description="output topic for object lists in the fixed frame",
        ),
        DeclareLaunchArgument("output_ego_imu_topic", default_value="~/ego_imu", description="output topic for ego IMU data"),
        DeclareLaunchArgument(
            "output_ego_odometry_topic",
            default_value="~/ego_odometry",
            description="output topic for ego odometry",
        ),
        DeclareLaunchArgument(
            "output_ego_vehicle_state_topic",
            default_value="~/ego_vehicle_state",
            description="output topic for ego vehicle state",
        ),
    ]

    args = [
        DeclareLaunchArgument("name", default_value="simulation_adapter", description="node name"),
        DeclareLaunchArgument("namespace", default_value="", description="node namespace"),
        DeclareLaunchArgument(
            "params",
            default_value=os.path.join(get_package_share_directory("simulation_adapter"), "config", "params.yml"),
            description="path to parameter file",
        ),
        DeclareLaunchArgument(
            "log_level", default_value="info", description="ROS logging level (debug, info, warn, error, fatal)"
        ),
        DeclareLaunchArgument("use_sim_time", default_value="true", description="use simulation clock"),
        DeclareLaunchArgument(
            "load_lanelet_map", default_value="true", description="automatically set lanelet2 map from simulation map info"
        ),
        *remappable_topics,
    ]

    nodes = [
        Node(
            package="simulation_adapter",
            executable="simulation_adapter",
            namespace=LaunchConfiguration("namespace"),
            name=LaunchConfiguration("name"),
            parameters=[LaunchConfiguration("params")],
            arguments=["--ros-args", "--log-level", LaunchConfiguration("log_level")],
            remappings=[(la.default_value[0].text, LaunchConfiguration(la.name)) for la in remappable_topics],
            output="screen",
            emulate_tty=True,
        )
    ]

    return LaunchDescription(
        [
            *args,
            SetParameter("use_sim_time", LaunchConfiguration("use_sim_time")),
            SetParameter("load_lanelet_map", LaunchConfiguration("load_lanelet_map")),
            *nodes,
        ]
    )
