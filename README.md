# simulation_its_adapter

This package contains the SimulationItsAdapterNode - a simple ROS 2 Node that converts incoming messages from the simulation_its_converter, e.g. [simulation_its_converter](https://gitlab.ika.rwth-aachen.de/fb-fi/simulation/carla/simulation_its_converter), to `fixed_frame_id` and `vehicle_frame_id` frame. Moreover, the node can load a lanelet map based on the currently reported simulation map info.

- [simulation\_its\_adapter](#simulation_its_adapter)
  - [Nodes](#nodes)
    - [simulation\_its\_adapter/SimulationItsAdapterNode](#simulation_its_adaptersimulationitsadapternode)
      - [Subscribed Topics](#subscribed-topics)
      - [Published Topics](#published-topics)
      - [Parameters](#parameters)
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
| `simulation_its_adapter` | `SimulationItsAdapterNode` | Connects a simulation core to an automated driving stack by converting simulation messages to internal frame conventions and requests a corresponding lanelet map |

### simulation_its_adapter/SimulationItsAdapterNode

#### Subscribed Topics

| Topic | Type | Description | 
| --- | --- | --- |
| `~/input_map_info` | `std_msgs::msg::String` | Current simulation map infos |
| `~/input_ego_data` | `perception_msgs::EgoData` | EgoData in the simulation_fixed_frame_id |
| `~/input_object_list` | `perception_msgs::ObjectList` | ObjectList in the simulation_fixed_frame_id |
| `~/input_trajectory` | `trajectory_planning_msgs::msg::Trajectory` | ObjectList in the simulation_fixed_frame_id |

#### Published Topics

| Topic | Type | Description |
| --- | --- | --- |
| `~/ego_data` | `perception_msgs::EgoData` | Ego data for vehicle_frame_id in the fixed_frame_id|
| `~/object_list` | `perception_msgs::ObjectList` | Object list in the vehicle_frame_id |
| `~/object_list_fixed` | `perception_msgs::ObjectList` | Object list in the fixed_frame_id |

#### Parameters

| Parameter | Type | Description |
| --- | --- | --- |
| map_server_name | string | Name of the map server. |
| simulation_fixed_frame_id | string | Name of the fixed frame id in the simulation. |
| fixed_frame_id | string | Name of the fixed frame id over time. |
| simulation_vehicle_frame_id | string | Name of the vehicle frame id in the simulation. |
| vehicle_frame_id | string | Name of the vehicle frame id. |
| simulation_vehicle_frame_id_to_vehicle_frame_id | float | Longitudinal offset from simulation_vehicle_frame_id to vehicle_frame_id. |

## Usage of docker-ros Images

### Available Images

| Tag | Description |
| --- | --- |
| ` ` | latest version |

### Default Command

```bash
ros2 launch simulation_its_adapter simulation_its_adapter.launch.py
```

### Launch Files

| Package | File | Path | Description |
| --- | --- | --- | --- |
| `simulation_its_adapter` | `simulation_its_adapter.launch.py` | `launch/` | Launches SimulationItsAdapterNode for ROS 2. |


### Configuration Files

| Package | File | Path | Description |
| --- | --- | --- | --- |
| ` `  |  |  |

### Additional Remarks

\-


## Official Documentation

\-
