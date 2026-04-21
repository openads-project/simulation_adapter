# simulation_adapter

This package contains the SimulationAdapter - a ROS 2 Node that connects the simulation to the automated driving stack by converting incoming messages from simulation cores, e.g. [carla_converter](https://gitlab.ika.rwth-aachen.de/fb-fi/simulation/carla/carla_converter) and [sumo_its_interface](https://gitlab.ika.rwth-aachen.de/fb-fi/simulation/sumo/sumo_its_interface), to `fixed_frame_id` and `vehicle_frame_id` frame. Moreover, the node can load a lanelet map based on the currently reported simulation map info.

- [simulation\_adapter](#simulation_adapter)
    - [Container Images](#container-images)
  - [`simulation_adapter`](#simulation_adapter-1)
    - [Subscribed Topics](#subscribed-topics)
    - [Published Topics](#published-topics)
    - [Parameters](#parameters)


### Container Images

| Description | Image:Tag | Default Command |
| --- | --- | --- |
| ROS 2 Node that connects the simulation to the automated driving stack | `gitlab.ika.rwth-aachen.de:5050/fb-fi/simulation/simulation_adapter:latest` | `ros2 launch simulation_adapter simulation_adapter.launch.py` |


## `simulation_adapter`

### Subscribed Topics

| Topic | Type | Description |
| --- | --- | --- |
| `~/input_map_info` | `std_msgs::msg::String` | Current simulation map info (latching QoS) |
| `~/input_ego_data` | `perception_msgs::msg::EgoData` | Ego data in `simulation_fixed_frame_id` |
| `~/input_object_list` | `perception_msgs::msg::ObjectList` | Object list in `simulation_fixed_frame_id` |
| `~/input_trajectory` | `trajectory_planning_msgs::msg::Trajectory` | Planned trajectory in any TF-reachable frame |

### Published Topics

| Topic | Type | Description |
| --- | --- | --- |
| `~/ego_data` | `perception_msgs::msg::EgoData` | Ego data in `fixed_frame_id`, shifted to the configured vehicle reference point and tagged as `REAR_AXLE_GROUND` |
| `~/ego_odometry` | `nav_msgs::msg::Odometry` | Ego pose in `fixed_frame_id` with `vehicle_frame_id` as child frame, published when `publish_ego_odometry=true` |
| `~/ego_vehicle_state` | `perception_msgs::msg::ObjectState` | Ego steering state containing `model_id`, `SteeringAngleAck`, and `SteeringAngleRateAck`, published when `publish_ego_vehicle_state=true` |
| `~/ego_imu` | `sensor_msgs::msg::Imu` | Ego IMU in `vehicle_frame_id` with only `linear_acceleration` populated, published when `publish_ego_imu=true` |
| `~/object_list` | `perception_msgs::msg::ObjectList` | Object list in `vehicle_frame_id` |
| `~/object_list_fixed` | `perception_msgs::msg::ObjectList` | Object list in `fixed_frame_id` |

### Parameters

| Parameter | Type | Default | Description |
| --- | --- | --- | --- |
| `map_server_name` | `string` | `/localization/ll2_map_server` | Name of the lanelet2 map server node |
| `load_lanelet_map` | `bool` | `true` | Automatically load the lanelet2 map based on the simulation map info |
| `simulation_fixed_frame_id` | `string` | `simulation_map` | Fixed frame id used by the simulation |
| `fixed_frame_id` | `string` | `map` | Fixed frame id used by the driving stack |
| `simulation_vehicle_frame_id` | `string` | `ego_vehicle` | Vehicle frame id used by the simulation |
| `vehicle_frame_id` | `string` | `base_link` | Vehicle frame id used by the driving stack |
| `publish_vehicle_frame_tf` | `bool` | `true` | Publish the static TF from `simulation_vehicle_frame_id` to `vehicle_frame_id` |
| `publish_ego_odometry` | `bool` | `false` | Publish ego odometry |
| `publish_ego_vehicle_state` | `bool` | `false` | Publish ego vehicle state |
| `publish_ego_imu` | `bool` | `false` | Publish ego IMU with linear acceleration only |
| `simulation_vehicle_frame_id_to_vehicle_frame_id` | `double` | `0.0` | Longitudinal offset (m) from `simulation_vehicle_frame_id` to `vehicle_frame_id` |
| `maps.simulation_maps` | `string[]` | `[campus]` | List of supported simulation map names (parallel to `maps.lanelet_files`) |
| `maps.lanelet_files` | `string[]` | `[/data/maps/aachen.osm]` | Lanelet2 map file paths (parallel to `maps.simulation_maps`) |
