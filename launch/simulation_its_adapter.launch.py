#!/usr/bin/env python3

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node, SetParameter
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution


def generate_launch_description():

  params_arg = DeclareLaunchArgument('params', default_value=PathJoinSubstitution([
      get_package_share_directory("simulation_its_adapter"), "config", "params.yml"])
  )

  node_name_arg = DeclareLaunchArgument('node_name', default_value='simulation_its_adapter')
  namespace_arg = DeclareLaunchArgument('namespace', default_value='')

  input_map_info_arg = DeclareLaunchArgument('input_map_info_topic', default_value='/carla_its_converter/map_info')
  input_ego_data_topic_arg = DeclareLaunchArgument('input_ego_data_topic', default_value='/carla_its_converter/ego_vehicle/ego_data')
  input_object_list_topic_arg = DeclareLaunchArgument('input_object_list_topic', default_value='/carla_its_converter/ego_vehicle/object_list')
  input_odometry_topic_arg = DeclareLaunchArgument('input_odometry_topic', default_value='/carla/ego_vehicle/odometry')
  input_trajectory_topic_arg = DeclareLaunchArgument('input_trajectory_topic', default_value='~/trajectory')

  output_ego_data_topic_arg = DeclareLaunchArgument('output_ego_data_topic', default_value='~/ego_data')
  output_object_list_topic_arg = DeclareLaunchArgument('output_object_list_topic', default_value='~/object_list')
  output_object_list_fixed_topic_arg = DeclareLaunchArgument('output_object_list_fixed_topic', default_value='~/object_list_fixed')

  use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value='true')
  set_ll2_map_from_simulation_arg = DeclareLaunchArgument('set_ll2_map_from_simulation', default_value='true')

  return LaunchDescription([
    params_arg,
    node_name_arg,
    namespace_arg,
    input_map_info_arg,
    input_ego_data_topic_arg,
    input_object_list_topic_arg,
    input_odometry_topic_arg,
    input_trajectory_topic_arg,
    output_ego_data_topic_arg,
    output_object_list_topic_arg,
    output_object_list_fixed_topic_arg,
    use_sim_time_arg,
    set_ll2_map_from_simulation_arg,
    SetParameter(name='use_sim_time', value=LaunchConfiguration('use_sim_time')),
    SetParameter(name='set_ll2_map_from_simulation', value=LaunchConfiguration('set_ll2_map_from_simulation')),
    Node(
      package="simulation_its_adapter",
      executable="simulation_its_adapter_node",
      name=LaunchConfiguration('node_name'),
      namespace=LaunchConfiguration('namespace'),
      output="screen",
      emulate_tty=True,
      parameters=[LaunchConfiguration('params')],
      remappings=[
          ("~/input_map_info", LaunchConfiguration('input_map_info_topic')),
          ("~/input_ego_data", LaunchConfiguration('input_ego_data_topic')),
          ("~/input_object_list", LaunchConfiguration('input_object_list_topic')),
          ("~/input_odometry", LaunchConfiguration('input_odometry_topic')),
          ("~/input_trajectory", LaunchConfiguration('input_trajectory_topic')),
          ("~/ego_data", LaunchConfiguration('output_ego_data_topic')),
          ("~/object_list", LaunchConfiguration('output_object_list_topic')),
          ("~/object_list_fixed", LaunchConfiguration('output_object_list_fixed_topic'))
      ]
    )
  ])