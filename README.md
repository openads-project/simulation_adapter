# carla_its_adapter

This package contains the CarlaItsAdapterNode - a simple ROS Node that converts incoming messages from the [carla-its-converter](https://gitlab.ika.rwth-aachen.de/fb-fi/simulation/carla/carla_its_converter) and publishes some of the [fb-fi defined ros messages](https://gitlab.ika.rwth-aachen.de/fb-fi/definitions) in map and base_link frame. Moreover the node is capable to call a service to load a lanelet-map by parsing the available OpenDRIVE map.

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
| `/carla/world_info` | `cm::CarlaWorldInfo` | World info of the CARLA environment |
| `/carla_its_converter/object_list/carla_map` | `pi::ObjectList` | Objects in the carla environment in ITS format|
| `/carla/ego_vehicle/odometry` | `nm::Odometry` | Odometry of the ego vehicle |

#### Published Topics

| Topic | Type | Description |
| --- | --- | --- |
| `/carla_its_adapter/object_list/map` | `perception_interfaces::ObjectList` | Object list in map frame |
| `/carla_its_adapter/object_list/base_link` | `pi::ObjectList` | Object list in base link frame |

#### Parameters

| Parameter | Type | Description |
| --- | --- | --- |
| fov_range | float | Maximum field of view range for objects in the base link frame. Only objects within the FOV will be published. |
| center_to_baselink | float | Distance between center of the vehicle and its base link. |


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
| `carla_its_adapter` | `carla_its_adapter.launch.py` | `launch/` | Launches CarlaItsAdapterNode for ROS2. |


### Configuration Files

| Package | File | Path | Description |
| --- | --- | --- | --- |
| ` `  |  |  |

### Additional Remarks

\-


## Official Documentation

\-
