#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node, SetParameter
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution


def generate_launch_description():

  node_name_arg = DeclareLaunchArgument('node_name', default_value='carla_its_adapter')
  namespace_arg = DeclareLaunchArgument('namespace', default_value='')

  input_ego_data_topic_arg = DeclareLaunchArgument('input_ego_data_topic', default_value='/carla_its_converter/ego_vehicle/ego_data')
  input_object_list_topic_arg = DeclareLaunchArgument('input_object_list_topic', default_value='/carla_its_converter/ego_vehicle/object_list')
  input_odometry_topic_arg = DeclareLaunchArgument('input_odometry_topic', default_value='/carla_its_converter/ego_vehicle/odometry')
  input_trajectory_topic_arg = DeclareLaunchArgument('input_trajectory_topic', default_value='~/trajectory')
  
  output_ego_data_topic_arg = DeclareLaunchArgument('output_ego_data_topic', default_value='~/ego_data')
  output_object_list_topic_arg = DeclareLaunchArgument('output_object_list_topic', default_value='~/object_list')
  output_object_list_map_topic_arg = DeclareLaunchArgument('output_object_list_topic_map', default_value='~/object_list_map')

  vehicle_frame_arg = DeclareLaunchArgument('vehicle_frame', default_value='base_link')
  center_to_base_link_arg = DeclareLaunchArgument('center_to_base_link', default_value='-1.2645')
  map_server_name_arg = DeclareLaunchArgument('map_server_name', default_value='/ll2_map_server')
  use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value='False')

  return LaunchDescription([
    node_name_arg,
    namespace_arg,
    input_ego_data_topic_arg,
    input_object_list_topic_arg,
    input_odometry_topic_arg,
    input_trajectory_topic_arg,
    output_ego_data_topic_arg,
    output_object_list_topic_arg,
    output_object_list_map_topic_arg,
    vehicle_frame_arg,
    use_sim_time_arg,
    SetParameter(name='use_sim_time', value=LaunchConfiguration('use_sim_time')),
    Node(
      package="carla_its_adapter",
      executable="carla_its_adapter_node",
      name=LaunchConfiguration('node_name'),
      namespace=LaunchConfiguration('namespace'),
      output="screen",
      emulate_tty=True,
      parameters=[LaunchConfiguration('params')],
      remappings=[
          ("~/input_ego_data", LaunchConfiguration('input_ego_data_topic')),
          ("~/input_object_list", LaunchConfiguration('input_object_list_topic')),
          ("~/input_odometry", LaunchConfiguration('input_odometry_topic')),
          ("~/input_trajectory", LaunchConfiguration('input_trajectory_topic')),
          ("~/ego_data", LaunchConfiguration('output_ego_data_topic')),
          ("~/object_list", LaunchConfiguration('output_object_list_topic')),
          ("~/object_list_map", LaunchConfiguration('output_object_list_topic_map'))
      ]
    )
  ])