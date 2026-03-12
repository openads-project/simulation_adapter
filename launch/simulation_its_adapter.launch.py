#!/usr/bin/env python3

import os

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetParameter


def generate_launch_description():

    remappable_topics = [
        DeclareLaunchArgument("input_map_info_topic", default_value="~/input_map_info"),
        DeclareLaunchArgument("input_ego_data_topic", default_value="~/input_ego_data"),
        DeclareLaunchArgument("input_object_list_topic", default_value="~/input_object_list"),
        DeclareLaunchArgument("input_trajectory_topic", default_value="~/input_trajectory"),
        DeclareLaunchArgument("output_ego_data_topic", default_value="~/ego_data"),
        DeclareLaunchArgument("output_object_list_topic", default_value="~/object_list"),
        DeclareLaunchArgument("output_object_list_fixed_topic", default_value="~/object_list_fixed"),
    ]

    args = [
        DeclareLaunchArgument("name", default_value="simulation_its_adapter", description="node name"),
        DeclareLaunchArgument("namespace", default_value="", description="node namespace"),
        DeclareLaunchArgument("params", default_value=os.path.join(get_package_share_directory("simulation_its_adapter"), "config", "params.yml"), description="path to parameter file"),
        DeclareLaunchArgument("log_level", default_value="info", description="ROS logging level (debug, info, warn, error, fatal)"),
        DeclareLaunchArgument("use_sim_time", default_value="true", description="use simulation clock"),
        DeclareLaunchArgument("set_ll2_map", default_value="true", description="automatically set lanelet2 map from simulation map info"),
        *remappable_topics,
    ]

    nodes = [
        Node(
            package="simulation_its_adapter",
            executable="simulation_its_adapter",
            namespace=LaunchConfiguration("namespace"),
            name=LaunchConfiguration("name"),
            parameters=[LaunchConfiguration("params")],
            arguments=["--ros-args", "--log-level", LaunchConfiguration("log_level")],
            remappings=[(la.default_value[0].text, LaunchConfiguration(la.name)) for la in remappable_topics],
            output="screen",
            emulate_tty=True,
        )
    ]

    return LaunchDescription([
        *remappable_topics,
        *args,
        SetParameter("use_sim_time", LaunchConfiguration("use_sim_time")),
        SetParameter("set_ll2_map", LaunchConfiguration("set_ll2_map")),
        *nodes,
    ])
