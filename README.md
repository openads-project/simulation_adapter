# carla_its_adapter

This package contains the CarlaItsAdapterNode - a simple ROS 2 Node that converts incoming messages from the [carla_its_converter](https://gitlab.ika.rwth-aachen.de/fb-fi/simulation/carla/carla_its_converter) to `map` and `base_link` frame. Moreover, the node is capable to call a service to load a lanelet-map  by parsing the available OpenDRIVE map.

- [Nodes](#nodes)
  - [carla_its_adapter/CarlaItsAdapterNode](#carla_its_adaptercarlait_adapternode)
- [Usage of docker-ros Images](#usage-of-docker-ros-images)
  - [Available Images](#available-images)
  - [Default Command](#default-command)
  - [Launch Files](#launch-files)
  - [Configuration Files](#configuration-files)
  - [Additional Remarks](#additional-remarks)
- [Official Documentation](#official-documentation)


## Nodes

| Package | Node | Description |
| --- | --- | --- |
| `carla_its_adapter` | `CarlaItsAdapterNode` | Converting carla-ros-messages to fb-fi defined its-messages |

### carla_its_adapter/CarlaItsAdapterNode

#### Subscribed Topics

| Topic | Type | Description | 
| --- | --- | --- |
| `/carla/world_info` | `carla_msgs::msg::CarlaWorldInfo` | World info of the CARLA environment |
| `~/input_ego_data` | `perception_msgs::EgoData` | EgoData in the carla map frame|
| `~/input_object_list` | `perception_msgs::ObjectList` | ObjectList in the carla map frame.|
| `~/input_odometry` | `nav_msgs::Odometry` | Odometry message from carla.|
| `~/input_trajectory` | `planning_msgs::Trajectory` | ObjectList in the carla map frame.|

#### Published Topics

| Topic | Type | Description |
| --- | --- | --- |
| `~/object_list/map` | `perception_msgs::ObjectList` | Object list in map frame |
| `~/object_list` | `perception_msgs::ObjectList` | Object list in vehicle_frame |

#### Parameters

| Parameter | Type | Description |
| --- | --- | --- |
| map_server_name | string | Name of the map server. |
| vehicle_frame | string | Name of the vehicle frame. |
| geo_center_to_vehicle_frame | float | Distance between center of the vehicle and its base_link. |


## Usage of docker-ros Images

### Available Images

| Tag | Description |
| --- | --- |
| ` ` | latest version |

### Default Command

```bash
ros2 launch carla_its_adapter carla_its_adapter.launch.py
```

### Launch Files

| Package | File | Path | Description |
| --- | --- | --- | --- |
| `carla_its_adapter` | `carla_its_adapter_node_launch.py` | `launch/` | Launches CarlaItsAdapterNode for ROS2. |


### Configuration Files

| Package | File | Path | Description |
| --- | --- | --- | --- |
| ` `  |  |  |

### Additional Remarks

\-


## Official Documentation

\-
